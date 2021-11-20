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


std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

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
        // cv::namedWindow("STEREO", CV_WINDOW_NORMAL);
        // cv::namedWindow(wname_or);
        // cv::imshow(wname_or, img);
        // std::cout<<"PRESS ANY KEY TO CONTINUE..."<<std::endl;
        // mouse_parameters_or.img = img.clone();
        // mouse_parameters_or.wname = wname_or;

        // cvSetMouseCallback( wname_or.c_str(), on_mouse, &mouse_parameters_or );
        // cv::waitKey(0);

        

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

        //Practica 3

        cv::Mat new_left;
        cv::Mat new_right;

        cv::cvtColor(img_left,new_left, CV_BGR2GRAY);
        // img_left.convertTo(new_left,CV_8UC1);
        
        cv::cvtColor(img_right,new_right, CV_BGR2GRAY);
        // img_right.convertTo(new_right,CV_8UC1);

        std::string ty =  type2str( img_left.type() );

        cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create();
        //cv::Mat disp(img_left.rows, img_left.cols, CV_8UC1), disparity;
        cv::Mat disp, disparity;
        sbm->compute(new_left,new_right,disp);

        // Converting disparity values to CV_32F from CV_16S
        disp.convertTo(disparity,CV_32F, 1.0);
        disparity=disparity/16.f;

        float B = sqrt((float)pow(st_parameters.Trns.at<double>(0,0),2) + (float)pow(st_parameters.Trns.at<double>(1,0),2) + (float)pow(st_parameters.Trns.at<double>(2,0),2));
        float f = (float)st_parameters.mtxL.at<double>(0,0);
        float cx = (float)st_parameters.mtxL.at<double>(0,2);
        float cy = (float)st_parameters.mtxL.at<double>(1,2);
        float X,Y,Z;

        std::vector<cv::Point3f> _3dpoints;
        // std::cout<<disparity<<std::endl;

        for(int x = 0; x<disparity.cols; x++){
            for(int y = 0; y<disparity.rows; y++){            
                if(disparity.at<float>(y,x) > 10.0){
                    Z = B*f/disparity.at<float>(y,x);
                    X = (x-cx)*Z/f;
                    Y = (y-cy)*Z/f;
                    _3dpoints.push_back(cv::Point3f(X,Y,Z));
                }
                else{
                    // _3dpoints.push_back(cv::Point3f(x,y,0));
                }
            }
        }

        writeToPCD(output_file,_3dpoints);

        // for (int c = 0; c < st_parameters.Trns.rows; c++)
        // {
        //     for (int d = 0; d < st_parameters.Trns.cols; d++)
        //     {
        //         std::cout << "row: " << c << " col: " << d << " " << st_parameters.Trns.at<double>(c, d) << std::endl;
        //     }
        // }
        // // Scaling down the disparity values and normalizing them
        // cv::Mat disparityNorm = (disparity/16.0f -
        // (float)minDisparity)/((float)numDisparities);

        // std::cout<<"HOLA"<<std::endl;


        
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
