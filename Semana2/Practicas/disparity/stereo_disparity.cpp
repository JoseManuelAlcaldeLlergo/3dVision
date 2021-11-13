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
#include "dirreader.h"

const cv::String keys =
    "{help h usage ? |      | print this message.}"
    "{@input        |<none>| path of the image.}"
    "{@calibration        |<none>| filename of the calibration file.}"    ;

void on_mouse( int event, int x, int y, int flags, void* param )
{
    OnMouseParams *params = (OnMouseParams*)param;

    cv::Mat aux = params->img.clone();
    
    if (event==CV_EVENT_MOUSEMOVE) 
    {
        cv::Point p1, p2;

        p1.x = 0;
        p1.y = y;
        p2.x = params->img.cols;
        p2.y = y;

        cv::line(aux, p1, p2, cv::Scalar(0,0,255), 1);

        cv::imshow(params->wname,aux);
    
    }
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
        if(argc != 3){
            std::cerr<<"Se le deben pasar dos argumentos al programa:\n\t ./stereo_calibrate directorio_imagenes fichero_salida"<<std::endl;
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
        cv::namedWindow(wname_or);
        cv::imshow(wname_or, img);
        std::cout<<"PRESS ANY KEY TO CONTINUE..."<<std::endl;
        cv::waitKey(0);

        mouse_parameters_or.img = img.clone();
        mouse_parameters_or.wname = wname_or;

        cvSetMouseCallback( wname_or.c_str(), on_mouse, &mouse_parameters_or );

        // // Dividimos la imagen en las dos que la componen
        cv::Mat img_left = img(cv::Range(0, img.rows), cv::Range(0, round(img.cols / 2)));
        cv::Mat img_right = img(cv::Range(0, img.rows), cv::Range(round(img.cols / 2), img.cols));

        rectifyStereoImages(st_parameters,img_left,img_right);

        //Imagen rectificada

        cv::Mat img_rect;
        cv::hconcat(img_left,img_right, img_rect);

        cv::namedWindow(wname_rect);
        cv::imshow(wname_rect, img_rect);

        mouse_parameters_rect.img = img_rect.clone();
        mouse_parameters_rect.wname = wname_rect;
        
        cvSetMouseCallback( wname_rect.c_str(), on_mouse, &mouse_parameters_rect );
        std::cout<<"PRESS ANY KEY TO FINISH..."<<std::endl;
        cv::waitKey(0);

        
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
