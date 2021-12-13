// ./stereo_sparse ../reconstruction/m001.jpg ../stereo_calibration.yml  outpu.pcd

#include <iostream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d.hpp>

#include "common_code.hpp"

const cv::String keys =
    "{help h usage ? |      | print this message.}"
    "{@im1        |<none>| path of the image.}"
    "{@im2        |<none>| path of the image.}"
    "{@calibration        |<none>| filename of the calibration file.}";

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;

    try
    {
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Stereo Calibration");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        if (argc != 4)
        {
            std::cerr << "Se le deben pasar cuatro argumentos al programa:\n\t ./fundamental_matrix img1.jpg img2.jpg calibraton.yml" << std::endl;
            return EXIT_FAILURE;
        }
        cv::Mat im1=cv::imread(argv[1],CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat im2=cv::imread(argv[2],CV_LOAD_IMAGE_GRAYSCALE);
        std::string calibration_file = parser.get<cv::String>("@calibration");

        
        
        auto fs = cv::FileStorage();
        fs.open(calibration_file, cv::FileStorage::READ);

        auto CP=readCameraParams(fs);
        cv::Mat und_im1 = removeDistortion(im1,CP).clone();
        cv::Mat und_im2 = removeDistortion(im2,CP).clone();
        cv::Mat F=fundamental(und_im1,und_im2);
        // __imshow("original1",im1,cv::Size(480,270));
        // __imshow("und_1",und_im1,cv::Size(480,270));
        // __imshow("original2",im2,cv::Size(480,270));
        // __imshow("und_2",und_im2,cv::Size(480,270));
        std::cout<<F<<std::endl;
        showEpipolar(und_im1,und_im2,CP.camera_matrix,F);
        while(cv::waitKey(0)!=27) ;

        
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
