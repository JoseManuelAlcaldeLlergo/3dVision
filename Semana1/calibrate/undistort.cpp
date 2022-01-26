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
    "{@output        |<none>| output image|video.}";

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;
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
    auto fourcc = parser.get<std::string>("fourcc");
    if (!parser.check())
    {
        parser.printErrors();
        return EXIT_FAILURE;
    }

    try
    {

        float error;
        cv::Size camera_size;
        cv::Mat K, dist_coeffs, rvec, tvec;

        auto fs = cv::FileStorage();
        fs.open(calib_fname, cv::FileStorage::READ);

        //TODO: First load the calibration parameters.
        fsiv_load_calibration_parameters(fs, camera_size, error, K, dist_coeffs, rvec, tvec);
        //

        cv::namedWindow("INPUT", cv::WINDOW_GUI_EXPANDED + cv::WINDOW_AUTOSIZE);
        cv::namedWindow("OUTPUT", cv::WINDOW_GUI_EXPANDED + cv::WINDOW_AUTOSIZE);
        //TODO

        //
        if (is_video)
        {
            //TODO

            int codec;
            cv::VideoCapture capture;
            capture.open(cv::samples::findFileOrKeep(input_fname));

            if (parser.has("fourcc") == true)
            {
                codec = cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
            }
            // Por defecto el codec del video de entrada
            else
            {
                int ex = static_cast<int>(capture.get(CV_CAP_PROP_FOURCC));
                char EXT[] = {ex & 0XFF , (ex & 0XFF00) >> 8,(ex & 0XFF0000) >> 16,(ex & 0XFF000000) >> 24, 0};
                std::cout<<"codec: "<<EXT<<std::endl;
                // codec = cv::VideoWriter::fourcc();
                codec = cv::VideoWriter::fourcc(EXT[0], EXT[1], EXT[2], EXT[3]);
            }

            cv::VideoWriter v_writer(output_fname, codec, 20, cv::Size(capture.get(CV_CAP_PROP_FRAME_WIDTH), capture.get(CV_CAP_PROP_FRAME_HEIGHT)), true);

            fsiv_undistort_video_stream(capture, v_writer, K, dist_coeffs, cv::INTER_LANCZOS4, "INPUT", "OUTPUT", 15);
            
            capture.release();
            v_writer.release();
            //
        }
        else
        {
            //TODO
            cv::Mat input = cv::imread(input_fname);
            cv::Mat output;

            fsiv_undistort_image(input, output, K, dist_coeffs);
            cv::imwrite(output_fname, output);
            //
        }
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
