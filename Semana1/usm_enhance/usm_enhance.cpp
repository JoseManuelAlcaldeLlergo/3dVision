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
    std::string trackbar;
};

float g_interactve;
int r_interactive;
int G;
cv::Mat in, out;

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

    in = user_data->input.clone();
    out = user_data->output.clone();

    //
    g_interactve = G/100.0;
    
    // std::cout<<"r= "<<r_interactive<<std::endl;
    // std::cout<<"g= "<<g_interactve<<std::endl;

    // user_data->input.convertTo(user_data->input, CV_32F, 1.0 / 255.0);
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
        in_ = user_data->input.clone();
    }

    out = fsiv_usm_enhance(in_, g_interactve, r_interactive, user_data->filter_type, user_data->circular, &user_data->mask);

    if (in.channels() == 3)
    {
        chs[2] = out.clone();
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

        cv::Mat origin_in = cv::imread(input_n, cv::IMREAD_UNCHANGED);
        cv::Mat origin_out = in.clone();
        cv::Mat mask = in.clone();

        if (origin_in.empty())
        {
            std::cerr << "Error: could not open input image '" << input_n
                      << "'." << std::endl;
            return EXIT_FAILURE;
        }

        cv::namedWindow("INPUT");
        cv::namedWindow("OUTPUT");
        cv::namedWindow("UNSHARP MASK");

        UserData user_data;

        /*Pasamos al modo interactivo todas las variables que puede necesitar*/
        user_data.input = origin_in.clone();
        user_data.output = origin_out.clone();
        user_data.r = r;
        user_data.g = g;
        user_data.filter_type = filter_type;
        user_data.circular = circular;
        user_data.input.convertTo(user_data.input, CV_32F, 1.0 / 255.0);
        if (interactive)
        {
            r_interactive = r;
            g_interactve = g;
            user_data.trackbar = "r";
            cv::createTrackbar("Radius", "OUTPUT", &r_interactive, 100, on_change, &user_data);
            user_data.trackbar = "g";
            //Reescalamos la G para poder usarla como entero en el trackbar
            G = g_interactve*100;
            cv::createTrackbar("Gain", "OUTPUT", &G, 200, on_change, &user_data);

        }

        std::vector<cv::Mat> chs;
        cv::Mat in_;

        if (user_data.input.channels() == 3)
        {
            cv::Mat hsv;
            cv::cvtColor(user_data.input, hsv, cv::COLOR_BGR2HSV);
            cv::split(hsv, chs);
            in_ = chs[2];
        }
        else
        {
            in_ = user_data.input;
        }

        user_data.output = fsiv_usm_enhance(in_, user_data.g, user_data.r, user_data.filter_type, user_data.circular, &mask);

        if (origin_in.channels() == 3)
        {
            chs[2] = user_data.output;
            cv::Mat hsv;
            cv::merge(chs, hsv);
            cv::cvtColor(hsv, user_data.output, cv::COLOR_HSV2BGR);
        }
        //}

        //Mostramos con in y out para que se guarden los cambios del modo interactivo
        // Inicializamos para que de primeras no estén vacias
        in = user_data.input.clone();
        out = user_data.output.clone();

        if(interactive){

        }
        cv::imshow("INPUT", in);
        cv::imshow("OUTPUT", out);
        cv::imshow("UNSHARP MASK", mask);

        std::cout << "Pulse ESC para cerrar sin guardar o cualquier otra tecla para cerrar y guardar la imagen..." << std::endl;

        int key = cv::waitKey(0) & 0xff;

        if (key != 27)
        {
            //imwrite trabaja con escala 0-255. Para guardar bien la imagen:
            out.convertTo(out, CV_8UC3, 255.0);
            if (!cv::imwrite(output_n, out))
            {
                std::cerr << "Error: could not save the result in file '"
                          << output_n << "'." << std::endl;
                return EXIT_FAILURE;
            }
        }

        std::cout << "\nFIN DEL PROGRAMA" << std::endl;
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
