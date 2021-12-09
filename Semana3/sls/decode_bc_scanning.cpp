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
    "{help h usage ? |      | print this message   }"
    "{v verbose      |0     | Verbose level. Value 0 means not log.}"
    "{a axis         |0     | Axis to decode 0:Y, 1:X, 2:both.}"
    "{@cparams       |<none>| Calibration parameters.}"
    "{@scanning      |<none>| Scanning.}"
    "{@output        |<none>| output wrl file.}"
    ;

int
main (int argc, char* const* argv)
{
    int retCode=EXIT_SUCCESS;

    try {

        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Decode a scanning using binary codes.");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }

        fsiv::__VerboseLevel = parser.get<int>("v");

        int axis = parser.get<int>("a");
        std::ofstream output (parser.get<std::string>("@output"));
        if (!output)
        {
            std::cerr << "Error: could not open to write the file ["
                      << parser.get<std::string>("@output") << "]." << std::endl;
            return EXIT_FAILURE;
        }

        fsiv::CParams cparams;
        if (!fsiv::load_calibration_parameters_from_file(
                    parser.get<std::string>("@cparams"), cparams))
        {
            std::cerr << "Error: could not read the calibrations parameters from file ["
                      << parser.get<std::string>("@cparams") << "]." << std::endl;
            return EXIT_FAILURE;
        }
        auto sc = fsiv::load<fsiv::BinaryCodeScanning>(parser.get<std::string>("@scanning"));
        if (sc == nullptr)
        {
            std::cerr << "Error: could not read the scanning from file ["
                      << parser.get<std::string>("@scanning") << "]." << std::endl;
            return EXIT_FAILURE;
        }

        if (fsiv::__VerboseLevel>0)
            fsiv::show_scanning(sc);

        cv::Mat grey_pos, grey_neg;
        cv::cvtColor(sc->seq[0], grey_pos, cv::COLOR_RGB2GRAY);
        cv::cvtColor(sc->seq[1], grey_neg, cv::COLOR_RGB2GRAY);
        cv::Mat mask = grey_pos>grey_neg+40;
        if (fsiv::__VerboseLevel>0)
        {
            cv::imshow("MASK", mask);
            cv::waitKey(0);
            cv::destroyWindow("MASK");
        }

        cv::Mat x_codes_, y_codes_;
        sc->decode_scanning(x_codes_, y_codes_);

        cv::Mat x_codes=cv::Mat::zeros(x_codes_.size(), x_codes_.type());
        cv::Mat y_codes=cv::Mat::zeros(y_codes_.size(), y_codes_.type());
        x_codes_.copyTo(x_codes, mask);
        y_codes_.copyTo(y_codes, mask);

        if (fsiv::__VerboseLevel>0)
        {
            cv::Mat x_codes_n, y_codes_n;
            cv::normalize(x_codes, x_codes_n, 0, 1, cv::NORM_MINMAX, CV_32F);
            cv::normalize(y_codes, y_codes_n, 0, 1, cv::NORM_MINMAX, CV_32F);
            cv::imshow("x_codes", x_codes_n);
            cv::imshow("y_codes", y_codes_n);
            cv::waitKey(0);
            cv::destroyWindow("x_codes");
            cv::destroyWindow("y_codes");
        }

        cv::Mat XYZ;        
        if (axis==0)
        {
            XYZ = fsiv::compute_line_plane_triangulation(y_codes, axis, cparams, mask);
        }
        else if (axis==1)
        {
            XYZ = fsiv::compute_line_plane_triangulation(x_codes, axis, cparams, mask);
        }
        else
            XYZ = fsiv::compute_line_line_triangulation(x_codes, y_codes, cparams, mask);

        mask = fsiv::clip_XYZ_data(XYZ, -0.5, 0.5, -0.5, 0.5, -0.25, 0.25, mask);
        fsiv::save_XYZ_to_vrml(output, XYZ, sc->seq[0], mask);
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
