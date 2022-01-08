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
    "{help h usage ? |      | print this message.}"
    "{v video        |      | the input is a video file.}"
    "{fourcc         |      | output video codec used, for example \"MJPG\". Default same as input.}"
    "{@intrinsics    |<none>| intrinsics parameters file.}"
    "{@input         |<none>| input image|video.}"
    "{@output        |<none>| output image|video.}"
    ;


int
main (int argc, char* const* argv)
{
    int retCode=EXIT_SUCCESS;
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Undistort an image or video file.");
    if (parser.has("help"))
    {
        parser.printMessage();
        return EXIT_SUCCESS;
    }
    auto is_video = parser.has("v");
    auto calib_fname = parser.get<std::string>("@intrinsics");
    auto input_fname = parser.get<std::string>("@input");
    auto output_fname = parser.get<std::string>("@output");
    if (!parser.check())
    {
        parser.printErrors();
        return EXIT_FAILURE;
    }

    try {

        float error;
        cv::Size camera_size;
        cv::Mat K, dist_coeffs, rvec, tvec;

        auto fs = cv::FileStorage();
        fs.open(calib_fname, cv::FileStorage::READ);

        //TODO: First load the calibration parameters.
        fsiv_load_calibration_parameters(fs,camera_size,error, K, dist_coeffs, rvec, tvec);
        //

        cv::namedWindow("INPUT", cv::WINDOW_GUI_EXPANDED+cv::WINDOW_AUTOSIZE);
        cv::namedWindow("OUTPUT", cv::WINDOW_GUI_EXPANDED+cv::WINDOW_AUTOSIZE);
        //TODO
       

        //
        if (is_video)
        {
            //TODO
            cv::Mat new_K, out_map1, out_map2;
            camera_size = cv::Size(640, 480); // El video que tenemos
            cv::initUndistortRectifyMap(K, dist_coeffs, cv::getOptimalNewCameraMatrix(K, dist_coeffs, camera_size, 1, camera_size, 0), new_K, camera_size, CV_32FC1, out_map1, out_map2 );
            
            

            cv::VideoCapture capture;
            capture.open(cv::samples::findFileOrKeep(input_fname));
            auto v_writer = cv::VideoWriter(output_fname, CV_FOURCC('H','2','6','4'), 15.0, camera_size, true);

            while(true){
                cv::Mat view, new_view;
                //bool blink = false;

                if( capture.isOpened() )
                {
                    cv::Mat view0;
                    capture >> view0;
                    view0.copyTo(view);
                    //camera_size = cv::Size(view.cols, view.rows);
                    //std::cout<<"cam_siz= "<<camera_size<<std::endl;
                    
                }

                if(view.empty())
                {
                    
                    break;//Fin while
                }
                cv::remap(view, new_view, out_map1, out_map2, cv::INTER_LINEAR);
                cv::imshow("Undistort",new_view);
                cv::waitKey(0);

                if(v_writer.isOpened()){
                    v_writer.write(new_view);
                }
            }
            //
        }
        else
        {
            //TODO
            cv::Mat input = cv::imread(input_fname);
            cv::Mat output;

            fsiv_undistort_image(input, output, K, dist_coeffs);
            cv::imwrite(output_fname ,output);
            //
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
