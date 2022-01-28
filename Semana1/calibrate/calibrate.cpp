/*!
  Esto es un esqueleto de programa para usar en las prácticas
  de Visión Artificial.

  Se supone que se utilizará OpenCV.

  Para compilar, puedes ejecutar:
    g++ -Wall -o esqueleto esqueleto.cc `pkg-config opencv --cflags --libs`

*/

// ./calibrate -verbose -c=6  -r=5  -s=0.04 out.yml ../data/logitech_000_001.png  ../data/logitech_000_002.png  ../data/logitech_000_003.png
// ./calibrate -verbose -c=6  -r=5  -s=0.04 out.yml -v=../data/

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
    "{help h usage ? |      | print this message.}"
    "{verbose        |      | activate verbose mode.}"
    "{i intrinsics   |      | Calibrate only extrinsics parameters. Using intrinsics from given file (-i=intr-file).}"
    "{s size         |<none>| square size.}"
    "{r rows         |<none>| number of board's rows.}"
    "{c cols         |<none>| number of board's cols.}"
    "{@output        |<none>| filename for output intrinsics file.}"
    "{@input1        |<none>| first board's view.}"
    "{@input2        |      | second board's view.}"
    "{@inputn        |      | ... n-idx board's view.}";

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;

    try
    {
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Calibrate the intrinsics parameters of a camera.");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        float square_size = parser.get<float>("s");
        int rows = parser.get<int>("r");
        int cols = parser.get<int>("c");
        bool verbose = parser.has("verbose");
        std::string output_fname = parser.get<cv::String>("@output");
        if (!parser.check())
        {
            parser.printErrors();
            return EXIT_FAILURE;
        }

        //Get the inputs.
        //find the second argument without '-' at begin.
        int input = 1;
        bool found = false;
        while (input < argc && !found)
            found = argv[input++][0] != '-';

        std::vector<std::string> input_fnames;
        for (; input < argc; ++input)
            input_fnames.push_back(std::string(argv[input]));

        float error;
        cv::Size camera_size;
        cv::Mat K, dist_coeffs, rvec, tvec;
        std::string wname = "TABLERO";
        auto board_Size = cv::Size(cols - 1, rows - 1);
        std::vector<std::vector<cv::Point2f>> _2dpoints;
        std::vector<std::vector<cv::Point3f>> _3dpoints;
        std::vector<cv::Point2f> corners;

        std::vector<cv::Point3f> _3d_corners = fsiv_generate_3d_calibration_points(board_Size, square_size);

        if (parser.has("i"))
        {
            //TODO
            //Make extrinsic calibration.

            std::string calib_fname = parser.get<std::string>("i");

            auto fs_r = cv::FileStorage();
            fs_r.open(calib_fname, cv::FileStorage::READ);

            fsiv_load_calibration_parameters(fs_r, camera_size, error, K, dist_coeffs, rvec, tvec);

            //Solo necesita una vista, por lo que cogemos la primera
            cv::Mat img = cv::imread(input_fnames[0]);
            camera_size = cv::Size(img.cols, img.rows);

            if (fsiv_find_chessboard_corners(img, board_Size, corners, wname.c_str()))
            {
                fsiv_compute_camera_pose(_3d_corners, corners, K, dist_coeffs, rvec, tvec);
            }

            auto fs_w = cv::FileStorage();
            fs_w.open(output_fname, cv::FileStorage::WRITE);
            fsiv_save_calibration_parameters(fs_w, camera_size, error, K, dist_coeffs, rvec, tvec);

            if (verbose)
            {
                cv::Mat img = cv::imread(input_fnames[0]);
                fsiv_draw_axes(img, K, dist_coeffs, rvec, tvec, square_size, 3);
                cv::imshow("EJES", img);
                cv::waitKey(0);
                //
            }
        }
        else
        {
            //Make an intrisic calibration.

            if (input_fnames.size() < 2)
            {
                std::cerr<<"Se necesitan al menos dos vistas para la calibración íntrinseca"<<std::endl;
                return EXIT_FAILURE;
            }

            cv::Size camera_size;
            std::vector<cv::Mat> tvecs;
            std::vector<cv::Mat> rvecs;

            for (size_t v = 0; v < input_fnames.size(); v++)
            {
                cv::Mat img = cv::imread(input_fnames[v]);
                camera_size = cv::Size(img.cols, img.rows);

                std::vector<cv::Point2f> corners;
                if (fsiv_find_chessboard_corners(img, board_Size, corners, wname.c_str()))
                {
                    _2dpoints.push_back(corners);
                    _3dpoints.push_back(_3d_corners);
                }
            }

            float error = fsiv_calibrate_camera(_2dpoints, _3dpoints, camera_size, K, dist_coeffs, &rvecs, &tvecs);
            std::cout << "Error de reproyección: " << error << std::endl;

            auto fs = cv::FileStorage();
            fs.open(output_fname, cv::FileStorage::WRITE);

            fsiv_save_calibration_parameters(fs, camera_size, error, K, dist_coeffs, rvecs[0], tvecs[0]);
            //

            if (verbose)
            {
                //TODO
                //Show WCS axis on each pattern view.
                for (size_t v = 0; v < input_fnames.size(); v++)
                {
                    cv::Mat img = cv::imread(input_fnames[v]);
                    fsiv_draw_axes(img, K, dist_coeffs, rvecs[v], tvecs[v], square_size, 3);
                    cv::imshow("EJES", img);
                    cv::waitKey(0);
                }

                //
            }
        }
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
