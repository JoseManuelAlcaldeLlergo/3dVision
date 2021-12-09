#include <iostream>
#include <sstream>
#include <fstream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "sls.hpp"

const cv::String keys =
    "{help h usage ? |      | Print this message.}"
    "{v verbose      |0     | Verbose level. Value 0 means not log.}"
    "{x_origin       |0     | Projector window x origin.}"
    "{y_origin       |0     | Projector window y origin.}"
    "{cap_params     |cap_params.yml| Capturing parameters.}"
    "{prj_size       |800x600| Projector geometry WxH.}"
    "{prj_board_size |5x4| Projected board geometry WxH.}"
    "{cam_idx        |-1 | Camera device index.}"
    "{@board_size     |<none>| Calibration board size WxH.}"
    "{@square_size    |<none>| Calibration square size in WCS units.}"
    "{@output        |<none>| SLS system calibration output filename.}"
    ;

bool
get_size(const std::string& value, cv::Size& size)
{
    bool was_ok = true;
    std::istringstream in_buffer(value);
    char sep;
    in_buffer >> size.width >> sep >> size.height;
    if (!in_buffer)
        was_ok = false;
    return was_ok;
}

int
main (int argc, char* const* argv)
{
    int retCode=EXIT_SUCCESS;

    try {

        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Calibrate a Structured Light System.");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }

        int verbose = parser.get<int>("v");
        int x_origin = parser.get<int>("x_origin");
        int y_origin = parser.get<int>("y_origin");
        int cam_idx = parser.get<int>("cam_idx");
        std::string cap_params_fname = parser.get<std::string>("cap_params");
        cv::Size prj_size;

        if (!get_size(parser.get<std::string>("prj_size"), prj_size))
        {
            std::cerr << "Error: Wrong CLI parameter 'prj_size'."
                      << std::endl;
            return EXIT_FAILURE;
        }
        cv::Size prj_board_size;
        if (!get_size(parser.get<std::string>("prj_board_size"), prj_board_size))
        {
            std::cerr << "Error: Wrong CLI parameter 'prj_board_size'."
                      << std::endl;
            return EXIT_FAILURE;
        }
        //We want interal points only.
        --prj_board_size.width;
        --prj_board_size.height;

        cv::Size calib_board_size;
        if (!get_size(parser.get<std::string>("@board_size"), calib_board_size))
        {
            std::cerr << "Error: Wrong CLI parameter '@calib_board_size'."
                      << std::endl;
            return EXIT_FAILURE;
        }
        //We want interal points only.
        --calib_board_size.width;
        --calib_board_size.height;

        float square_size = parser.get<float>("@square_size");        
        std::string output_fname = parser.get<std::string>("@output");
        cv::FileStorage capt_params;
        capt_params.open(parser.get<std::string>("cap_params"), cv::FileStorage::READ);
        if (!capt_params.isOpened())
        {
            std::cerr << "Error: could not load capturing paramaters from ["
                      << parser.get<std::string>("cap_params")
                      << "]. Using defaults values."
                      << std::endl;
        }

        if (!parser.check())
        {
            parser.printErrors();
            return EXIT_FAILURE;
        }

        auto calib_patterns = fsiv::CalibrationPatternSeq::create(prj_size, prj_board_size);

        bool was_ok = true;
        bool go_out = false;
        std::ostringstream out_buffer;
        fsiv::Projector prj (x_origin, y_origin);
        fsiv::Capturer capt(cam_idx, capt_params);
        if (!capt.is_opened())
        {
            std::cerr << "Error: could not open the capturing device with idx: "
                      << cam_idx << '.' << std::endl;
            return EXIT_FAILURE;
        }
        int scan=0;
        int key=0;
        std::vector<cv::Point3f> object_points =
                fsiv::create_calibration_object_points(calib_board_size, square_size);
        std::vector<std::vector<cv::Point2f>> projector_2d_points;
        std::vector<std::vector<cv::Point2f>> camera_2d_points;
        std::vector<std::vector<cv::Point3f>> obj_3d_points;
        cv::Size camera_size;
        while (! go_out)
        {
            //Mostramos video en vivo para que configurar una nueva pose
            //del sistema.
            out_buffer.str("");
            out_buffer << "["<<scan+1<<"] Ajusta. Pulsa tecla (Esc aborta)";
            std::string wnd_name = out_buffer.str();
            prj.project(calib_patterns->seq[0], 20);
            was_ok = capt.show_live_video(wnd_name.c_str(), &key);
            cv::destroyWindow(wnd_name);
            if (!was_ok || (key&0xff) == 27)
                go_out = true;
            else
            {
                auto scanning = calib_patterns->clone();
                was_ok = capt.scan_pattern_sequence(prj, scanning);
                if (was_ok)
                {
                    camera_size = scanning->seq[0].size();
                    cv::Mat img_prj;
                    cv::cvtColor(scanning->seq[0], img_prj, cv::COLOR_BGR2GRAY);
                    //img_prj = np.amax(scanning[0], axis=2)
                    std::vector<cv::Point2f> corners;
                    was_ok = fsiv::find_chessboard(img_prj, prj_board_size,
                                                   corners, "PROJECTOR_CHESSBOARD");
                    if (was_ok)
                    {
                        //Buscamos el tablero de calibración en la imagen capturada por la cámara
                        cv::Mat img_cam;
                        cv::cvtColor(scanning->seq[1], img_cam, cv::COLOR_BGR2GRAY);
                        std::vector<cv::Point2f> cam_corners;
                        was_ok = fsiv::find_chessboard(img_cam,
                                calib_board_size, cam_corners, "CAMERA_CHESSBOARD");
                        if (was_ok)
                        {
                            //Creamos la imagen "virtual" del tablero como se vería si el projector
                            //fuera una camara.
                            cv::Mat H = cv::findHomography(corners, calib_patterns->corners);
                            cv::Mat virtual_img;
                            cv::warpPerspective(img_cam, virtual_img, H,
                                                calib_patterns->prj_size);
                            std::vector<cv::Point2f> prj_corners;
                            was_ok = fsiv::find_chessboard(virtual_img,
                                                           calib_board_size,
                                                           prj_corners,
                                                           "PRJ_VIRTUAL_CHESSBOARD");
                            if (was_ok)
                            {
                                projector_2d_points.push_back(prj_corners);
                                camera_2d_points.push_back(cam_corners);
                                obj_3d_points.push_back(object_points);
                                scan++;
                            }
                        }
                    }
                }
                else
                {
                    std::cerr << "Error: could not complete the scanning."
                              << std::endl;
                    go_out = true;
                }

            }
        }

        if (scan>=2)
        {
            std::cout << "Calibrating with " << scan << " views." << std::endl;
            cv::Mat K_cam, dc_cam;
            std::vector<cv::Mat> rvecs_cam, tvecs_cam;
            float cam_error;
            cam_error = cv::calibrateCamera(obj_3d_points, camera_2d_points,
                                            camera_size, K_cam, dc_cam,
                                            rvecs_cam, tvecs_cam);
            cv::Mat K_prj, dc_prj;
            std::vector<cv::Mat> rvecs_prj, tvecs_prj;
            float prj_error;
            prj_error = cv::calibrateCamera(obj_3d_points, projector_2d_points,
                                            prj_size, K_prj, dc_prj,
                                            rvecs_prj, tvecs_prj);

            std::cerr << "Reprojection errors camera: " << cam_error
                      << " projector: " << prj_error << std::endl;


            fsiv::CParams cparams;
            cparams.cam_size = camera_size;
            cparams.cam_K = K_cam;
            cparams.cam_D = dc_cam;
            cparams.cam_rvec = rvecs_cam.back();
            cparams.cam_tvec = tvecs_cam.back();

            cparams.prj_size = prj_size;
            cparams.prj_K = K_prj;
            cparams.prj_D = dc_prj;
            cparams.prj_rvec = rvecs_prj.back();
            cparams.prj_tvec = tvecs_prj.back();


            if (!fsiv::save_calibration_parameters_to_file(output_fname,
                                                          cparams))
            {
                std::cerr << "Error: could not write into [" <<
                             output_fname << "]." << std::endl;
                return EXIT_FAILURE;
            }
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
