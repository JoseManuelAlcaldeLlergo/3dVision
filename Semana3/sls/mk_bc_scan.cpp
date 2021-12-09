#include <iostream>
#include <fstream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

#include "sls.hpp"

const cv::String keys =
    "{help usage ?   |      | print this message   }"
    "{v verbose      |0     | Verbose level. Value 0 means not log.}"
    "{lsb            |2     | Number of less significative bits not codified.}"
    "{not_inversed   |      | Not generate inversed patterns.}"
    "{gray_codec     |      | Use Gray codec}"
    "{a axis         |<none>| Axis to be codified. 0->X, 1->Y, 2->both.}"
    "{w widht        |<none | Projector's width.}"
    "{h height       |-1    | Projector's height.}"
    "{@output        |<none>| Output scanning file.}"
    ;

int
main (int argc, char* const* argv)
{
    int retCode=EXIT_SUCCESS;

    try {

        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Create a binary coded pattern scan sequence.");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }

        int verbose = parser.get<int>("v");
        bool use_inversed = !parser.has("not_inversed");
        bool use_gray_codec = parser.has("gray_codec");
        int axis = parser.get<int>("a");
        int remove_lsb = parser.get<int>("lsb");
        int width = parser.get<int>("w");
        int height = parser.get<int>("h");
        std::string output_fname = parser.get<std::string>("@output");

        auto sc = fsiv::BinaryCodeScanning::create(cv::Size(width, height),
                                                         axis, remove_lsb,
                                                         use_inversed,
                                                         use_gray_codec, 0, 255);
        if (!sc->save(output_fname))
        {
            std::cerr << "Error: could not write into ["
                      << output_fname << "]." << std::endl;
            return EXIT_FAILURE;
        }

        if (verbose>0)
            fsiv::show_scanning(sc);
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}

