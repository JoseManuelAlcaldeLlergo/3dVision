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
//#include <opencv2/calib3d/calib3d.hpp>

#include "common_code.hpp"

const cv::String keys =
    "{help h usage ? |      | print this message.}"
    "{i interactive  |      | Interactive mode.}"
    "{r radius       |1     | Window's radius. Default 1.}"
    "{g gain         |1.0   | Enhance's gain. Default 1.0}"
    "{c circular     |      | Use circular convolution.}"
    "{f filter       |0     | Filter type: 0->Box, 1->Gaussian. Default 0.}"
    "{@input         |<none>| input image.}"
    "{@output        |<none>| output image.}";

/**
 * @brief Application State.
 * Use this structure to maintain the state of the application
 * that will be passed to the callbacks.
 */
struct UserData
{
    cv::Mat input;
    cv::Mat output;
    cv::Mat mask;
    int r;
    double g;
    int filter_type;
    bool circular;
};

/** @brief Standard trackbar callback
 * Use this function an argument for cv::createTrackbar to control
 * the trackbar changes.
 *
 * @arg v give the trackbar position.
 * @arg user_data allow to pass user data to the callback.
 */
void on_change(int v, void *user_data_)
{
    UserData *user_data = static_cast<UserData *>(user_data_);

    cv::Mat in, out;

    user_data->r = v;

    in = user_data->input;
    out = user_data->output;

    in.convertTo(in, CV_32F, 1.0 / 255.0);
    std::vector<cv::Mat> chs;
    cv::Mat in_;

    if (in.channels() == 3)
    {
        cv::Mat hsv;
        cv::cvtColor(in, hsv, cv::COLOR_BGR2HSV);
        cv::split(hsv, chs);
        in_ = chs[2];
    }
    else
    {
        in_ = in;
    }

    out = fsiv_usm_enhance(in_, user_data->g, user_data->r, user_data->filter_type, user_data->circular, &user_data->mask);

    if (in.channels() == 3)
    {
        chs[2] = out;
        cv::Mat hsv;
        cv::merge(chs, hsv);
        cv::cvtColor(hsv, out, cv::COLOR_HSV2BGR);
    }

    cv::imshow("INPUT", in);
    cv::imshow("OUTPUT", out);
    cv::imshow("UNSHARP MASK", user_data->mask);
    // cv::waitKey(0);

    //
}

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;

    try
    {
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Apply an unsharp mask enhance to an image.");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }

        cv::String input_n = parser.get<cv::String>("@input");
        cv::String output_n = parser.get<cv::String>("@output");
        if (!parser.check())
        {
            parser.printErrors();
            return EXIT_FAILURE;
        }

        //TODO
        double g = 1.0;
        int r = 1;
        int filter_type = 1;

        if (parser.has("gain"))
            g = parser.get<double>("gain");
        if (parser.has("radius"))
            r = parser.get<int>("radius");
        if (parser.has("filter") && (parser.get<int>("filter") == 0 || parser.get<int>("filter") == 1))
            filter_type = parser.get<int>("filter");

        bool circular = parser.has("circular");
        bool interactive = parser.has("interactive");

        //

        cv::Mat in = cv::imread(input_n, cv::IMREAD_UNCHANGED);
        cv::Mat out = in.clone();
        cv::Mat mask = in.clone();

        if (in.empty())
        {
            std::cerr << "Error: could not open input image '" << input_n
                      << "'." << std::endl;
            return EXIT_FAILURE;
        }

        cv::namedWindow("INPUT");
        cv::namedWindow("OUTPUT");
        cv::namedWindow("UNSHARP MASK");

        UserData user_data;

        /**/
        if (interactive)
        {
            user_data.input = in;
            user_data.output = out;
            user_data.r = r;
            user_data.g = g;
            user_data.filter_type = filter_type;
            user_data.circular = circular;
            cv::createTrackbar("Radius", "INPUT", &r, 100, on_change, &user_data);
        }
        else
        {
            in.convertTo(in, CV_32F, 1.0 / 255.0);
            std::vector<cv::Mat> chs;
            cv::Mat in_;

            if (in.channels() == 3)
            {
                cv::Mat hsv;
                cv::cvtColor(in, hsv, cv::COLOR_BGR2HSV);
                cv::split(hsv, chs);
                in_ = chs[2];
            }
            else
            {
                in_ = in;
            }

            out = fsiv_usm_enhance(in_, g, r, filter_type, circular, &mask);

            if (in.channels() == 3)
            {
                chs[2] = out;
                cv::Mat hsv;
                cv::merge(chs, hsv);
                cv::cvtColor(hsv, out, cv::COLOR_HSV2BGR);
            }
        }

        cv::imshow("INPUT", in);
        cv::imshow("OUTPUT", out);
        cv::imshow("UNSHARP MASK", mask);

        int k = cv::waitKey(0) & 0xff;
        if (k != 27)
            cv::imwrite(output_n, out);
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
