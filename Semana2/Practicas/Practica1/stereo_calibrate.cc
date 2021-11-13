#include <iostream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

#include "common_code.hpp"
#include "dirreader.h"

const cv::String keys =
    "{help h usage ? |      | print this message.}"
    "{@output        |<none>| filename for output file.}"
    "{@input        |<none>| images directory.}";

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;

    try
    {
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("P1");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        int rows = 6;
        int cols = 8;

        std::string output_fname = parser.get<cv::String>("@output");

        cv::Size board_size = cv::Size(cols - 1, rows - 1);
        double square_size = 0.02875;//size of each square


        std::string wname = "Stereo";

        cv::namedWindow(wname);

        std::vector<std::vector<cv::Point3f>> _3d_points;
        std::vector<std::vector<cv::Point2f>> _2d_points;

        DirReader Dir;
        auto files=Dir.read(parser.get<cv::String>("@input"),".jpg",DirReader::Params(true));

        //Obtenemos los puntos 3d
        std::vector<cv::Point3f> _3d_corners = fsiv_generate_3d_calibration_points(board_size, square_size);

        // for(int i = 0; i < files.size(); i++){
          
        // }
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}