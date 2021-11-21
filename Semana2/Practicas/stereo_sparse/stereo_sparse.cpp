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
#include <opencv2/calib3d.hpp>

#include "common_code.hpp"

const cv::String keys =
    "{help h usage ? |      | print this message.}"
    "{@input        |<none>| path of the image.}"
    "{@calibration        |<none>| filename of the calibration file.}"
    "{@output        |<none>| PCD output file.}"    ;


int
main (int argc, char* const* argv)
{
    int retCode=EXIT_SUCCESS;

    try {        
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Stereo Calibration");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        if(argc != 4){
            std::cerr<<"Se le deben pasar tres argumentos al programa:\n\t ./stereo_disparity stereo_image calibration.yml out.pcd"<<std::endl;
            return EXIT_FAILURE;
        }
        
        cv::Mat cam_mat_left, cam_mat_right, dist_coef_left, dist_coef_right, R, T, E, F; 
        StereoParams st_parameters;
        OnMouseParams mouse_parameters_or;
        OnMouseParams mouse_parameters_rect;
        std::string wname_or = "Original";
        std::string wname_rect = "Rectification";
        
        float error;
        cv::Size camera_size = cv::Size(0,0);
        std::string img_name = parser.get<cv::String>("@input");
        std::string calibration_file = parser.get<cv::String>("@calibration");
        std::string output_file = parser.get<cv::String>("@output");

        if(!IsPathExist(img_name)){
            std::cerr<<"No existe el la imagen <"<<img_name<<">"<<std::endl;
            return EXIT_FAILURE;
        }
        if(!IsPathExist(calibration_file)){
            std::cerr<<"No existe el fichero de calibración <"<<calibration_file<<">"<<std::endl;
            return EXIT_FAILURE;
        }

        
        auto fs = cv::FileStorage();
        fs.open(calibration_file, cv::FileStorage::READ);
        load_calibration_parameters(fs, camera_size, error, st_parameters.mtxL, st_parameters.mtxR, st_parameters.distL, st_parameters.distR, st_parameters.Rot, st_parameters.Trns, st_parameters.Emat, st_parameters.Fmat);

        //Imagen original
        cv::Mat img = cv::imread(img_name);
      

        // // Dividimos la imagen en las dos que la componen
        cv::Mat img_left = img(cv::Range(0, img.rows), cv::Range(0, round(img.cols / 2)));
        cv::Mat img_right = img(cv::Range(0, img.rows), cv::Range(round(img.cols / 2), img.cols));

        rectifyStereoImages(st_parameters,img_left,img_right);

        //Imagen rectificada

        cv::Mat img_rect;
        cv::hconcat(img_left,img_right, img_rect);

        cv::namedWindow(wname_rect);
        cv::imshow(wname_rect, img_rect);

        // mouse_parameters_rect.img = img_rect.clone();
        // mouse_parameters_rect.wname = wname_rect;
        
        // cvSetMouseCallback( wname_rect.c_str(), on_mouse, &mouse_parameters_rect );
        // std::cout<<"PRESS ANY KEY TO FINISH..."<<std::endl;
        cv::waitKey(0);

        //Practica 4

        std::vector<cv::KeyPoint> keypoints_query,keypoints_train;
        cv::Mat descriptors_query, descriptors_train;
        std::vector<cv::DMatch> matches;
        cv::Mat matchesImage;

        auto Detector=cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB,  0,  3, 1e-4f );
        Detector ->detectAndCompute(img_left, cv::Mat(), keypoints_query, descriptors_query);
        Detector ->detectAndCompute(img_right, cv::Mat(), keypoints_train, descriptors_train);
        auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        matcher->match(descriptors_query, descriptors_train, matches, cv::Mat());

        std::cout<<matches.size()<<" matches"<<std::endl;
        for(unsigned int i = 0; i<matches.size(); i++){
            std::cout<<matches[i].distance<<std::endl;
        }

        cv::drawMatches(img_left,keypoints_query, img_right, keypoints_train, matches, matchesImage);

        cv::imshow("MATCHES", matchesImage);
        cv::waitKey(0);
        cv::imshow("DescQ", descriptors_query);
        cv::waitKey(0);
        cv::imshow("DescT", descriptors_train);

        cv::waitKey(0);

        
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
