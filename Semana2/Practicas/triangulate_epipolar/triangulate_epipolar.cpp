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
    "{@calibration        |<none>| filename of the calibration file.}"
    "{@output           |<none>|    output filename}";

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;

    try
    {
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Triangulate epipolar");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        if (argc != 5)
        {
            std::cerr << "Se le deben pasar cinco argumentos al programa:\n\t ./fundamental_matrix img1.jpg img2.jpg calibraton.yml out.pcd" << std::endl;
            return EXIT_FAILURE;
        }

        std::string im1_file = parser.get<cv::String>("@im1");
        std::string im2_file = parser.get<cv::String>("@im2");
        std::string output_file = parser.get<cv::String>("@output");
        std::string cal_file = parser.get<cv::String>("@calibration");

        cv::Mat im1 = cv::imread(im1_file, CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat im2 = cv::imread(im2_file, CV_LOAD_IMAGE_GRAYSCALE);
        std::string calibration_file = parser.get<cv::String>("@calibration");

        auto fs = cv::FileStorage();
        fs.open(calibration_file, cv::FileStorage::READ);


        auto CP = readCameraParams(fs);

        // std::cout<<CP.camera_matrix<<std::endl;
        // std::cout<<CP.dist_coefs<<std::endl;
        // std::cout<<CP.R<<std::endl;
        // std::cout<<CP.t<<std::endl;

        cv::Mat und_im1 = removeDistortion(im1, CP).clone();
        cv::Mat und_im2 = removeDistortion(im2, CP).clone();
        cv::Mat F = fundamental(und_im1, und_im2);
        // std::cout << F << std::endl;

        std::vector<cv::Point3f> vpoints = Triangulate(und_im1, und_im2, F, CP);
        writeToPCD(argv[4], vpoints);
        while (cv::waitKey(0) != 27)
            ;
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
