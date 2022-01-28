/*!
  Esto es un esqueleto de programa para usar en las prácticas
  de Visión Artificial.

  Se supone que se utilizará OpenCV.

  Para compilar, puedes ejecutar:
    g++ -Wall -o esqueleto esqueleto.cc `pkg-config opencv --cflags --libs`

*/

#include <iostream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "common_code.hpp"

const cv::String keys =
    "{help h usage ? |      | print this message   }"
    "{m              |      | draw a 3d model else draw XYZ axis.}"
    "{c              |      | The input is a camera idx.}"
    "{i              |      | Render an image on the board. -i=img.png}"
    "{v              |      | Render a video on the board. -v=video.avi}"
    "{@rows          |<none>| board rows.}"
    "{@cols          |<none>| board cols.}"
    "{@size          |<none>| board squared side size.}"
    "{@intrinsics    |<none>| intrinsics filename.}"
    "{@input         |<none>| input stream (filename or camera idx)}";

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;

    try
    {

        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Do augmented reality on a input stream.");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        int rows = parser.get<int>("@rows");
        int cols = parser.get<int>("@cols");
        cv::Size board_size(cols - 1, rows - 1);
        float size = parser.get<float>("@size");
        std::string intrinsics_file = parser.get<std::string>("@intrinsics");
        bool is_camera = parser.has("c");
        bool plot_model = parser.has("m");
        int camera_idx = -1;
        std::string input_file;
        if (is_camera)
            camera_idx = parser.get<int>("@input");
        else
            input_file = parser.get<std::string>("@input");

        cv::Mat projected_image;
        if (parser.has("i"))
        {
            projected_image = cv::imread(parser.get<std::string>("i"));
            if (projected_image.empty())
            {
                std::cerr << "Error: could not open image ["
                          << parser.get<std::string>("i") << "]." << std::endl;
                return EXIT_FAILURE;
            }
        }

        cv::VideoCapture projected_video;
        if (parser.has("v"))
        {
            if (!projected_video.open(parser.get<std::string>("v")))
            {
                std::cerr << "Error: could not open video ["
                          << parser.get<std::string>("v") << "]." << std::endl;
                return EXIT_FAILURE;
            }
        }

        if (!parser.check())
        {
            parser.printErrors();
            return EXIT_FAILURE;
        }

        cv::VideoCapture cap;
        if (is_camera)
            cap.open(camera_idx);
        else
            cap.open(input_file);

        if (!cap.isOpened())
        {
            std::cerr << "Error: could not open the input stream!" << std::endl;
            return EXIT_FAILURE;
        }

        //TODO
        //Load camera calibration parameters.
        auto fs_r = cv::FileStorage();
        fs_r.open(intrinsics_file, cv::FileStorage::READ);

        cv::Mat camera_matrix, dist_coefs, rvec, tvec;
        cv::Size camera_size;
        float error;

        fsiv_load_calibration_parameters(fs_r, camera_size, error, camera_matrix, dist_coefs, rvec, tvec);

        //Compute the 3d coordinates of the board corners.
        std::string wname = "TABLERO";
        cv::namedWindow(wname, cv::WINDOW_GUI_EXPANDED);
        cv::namedWindow(wname, CV_WINDOW_NORMAL);
        cv::Mat board_img = cv::imread(input_file);

        std::vector<std::vector<cv::Point2f>> _2d_points;
        std::vector<std::vector<cv::Point3f>> _3d_points;
        std::vector<cv::Point2f> corners;

        std::vector<cv::Point3f> _3d_corners = fsiv_generate_3d_calibration_points(board_size, size);

        if (fsiv_find_chessboard_corners(board_img, board_size, corners, wname.c_str()))
        {
            _2d_points.push_back(corners);
            _3d_points.push_back(_3d_corners);
        }
        else{
            std::cerr<<"No se encontaron esquinas en el tablero, pruebe otra imagen"<<std::endl;
            return EXIT_FAILURE;
        }

        fsiv_compute_camera_pose(_3d_points[0], _2d_points[0], camera_matrix, dist_coefs, rvec, tvec);

        if (parser.has("i"))
        {
            fsiv_project_image(projected_image, board_img, board_size, _2d_points[0]);

            // Si -m se dibuja el modelo 3d, si no los ejes
            if (plot_model)
                fsiv_draw_3d_model(board_img, camera_matrix, dist_coefs, rvec, tvec, size);
            else
                fsiv_draw_axes(board_img, camera_matrix, dist_coefs, rvec, tvec, size, 3);

            cv::imshow(wname, board_img);
            cv::waitKey(0);
        }

        else if (parser.has("v"))
        {
            cv::Mat input_frame;

            int wait_time = (is_camera ? 20 : 1000.0 / 25.0); //for a video file we use 25fps.
            int key = 0;

            std::cout<<"Pulse ESC para salir..."<<std::endl;
            while (key != 27)
            {

                projected_video >> input_frame;

                // Cuando termine el video se cierra
                if (input_frame.empty())
                    break;

                // Proyectamos cada frame sobre nuestro tablero
                fsiv_project_image(input_frame, board_img, board_size, _2d_points[0]);

                // Si -m se dibuja el modelo 3d, si no los ejes
                if (plot_model)
                    fsiv_draw_3d_model(board_img, camera_matrix, dist_coefs, rvec, tvec, size);
                else
                    fsiv_draw_axes(board_img, camera_matrix, dist_coefs, rvec, tvec, size, 3);

                //
                cv::imshow(wname, board_img);
                key = cv::waitKey(wait_time) & 0xff;
            }
        }

        cv::destroyAllWindows();
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
