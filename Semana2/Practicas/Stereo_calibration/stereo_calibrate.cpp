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
    "{@input        |<none>| input images directory.}"
    "{@output        |<none>| filename for output file.}"    ;

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
        int rows = 6;
        int cols = 8;
        std::string files_dir = parser.get<cv::String>("@input");
        std::string output_fname = parser.get<cv::String>("@output");

        if(!IsPathExist(files_dir)){
            std::cerr<<"No existe el directorio de imágenes <"<<files_dir<<">"<<std::endl;
            return EXIT_FAILURE;
        }

        const cv::Size board_size = cv::Size(cols - 1, rows - 1);
        float square_size = 0.02875;//size of each square


        std::string wname = "Stereo";

        cv::namedWindow(wname);

        std::vector<std::vector<cv::Point3f>> _3d_points;
        std::vector<std::vector<cv::Point2f>> _2d_points_left;
        std::vector<std::vector<cv::Point2f>> _2d_points_right;

        cv::Size camera_size = cv::Size(0,0);
        cv::Size camera_size_old = cv::Size(0,0);

        DirReader Dir;
        auto files=Dir.read(files_dir,".jpg",DirReader::Params(true));
        
        //Obtenemos los puntos 3d
        std::vector<cv::Point3f> _3d_corners = generate_3d_calibration_points(board_size, square_size);

        for (unsigned int i = 0; i<files.size(); i++) {
            // std::cout<<files[i]<<std::endl;
            cv::Mat img = cv::imread(files[i]);
            // // Dividimos la imagen en las dos que la componen
            cv::Mat img_left = img(cv::Range(0, img.rows), cv::Range(0, round(img.cols / 2)));

            camera_size = cv::Size(img_left.cols, img_left.rows);

            cv::Mat img_right = img(cv::Range(0, img.rows), cv::Range(round(img.cols / 2), img.cols));

            // Descomentar para ver las imágenes separadas
            // cv::imshow("Original",img);
            // cv::imshow("Left",img_left);
            // cv::imshow("RIGHT",img_right);
            // cv::waitKey(0);

            camera_size = cv::Size(img_left.cols, img_left.rows);
            if((camera_size_old != cv::Size(0,0)) && camera_size_old != camera_size){
                std::cerr<<"Las imágenes deben tener las mismas dimensiones"<<std::endl;
                return EXIT_FAILURE;
            }

            camera_size_old = camera_size;
            std::vector<cv::Point2f> corners_left;
            std::vector<cv::Point2f> corners_right;

            //Aplica findChessboardCorners y cornersubpix y si los encuentra en las dos fotos rellenamos los vectores
            if (find_chessboard_corners(img_left, board_size, corners_left, wname.c_str()) && find_chessboard_corners(img_right, board_size, corners_right, wname.c_str())) {
                _2d_points_left.push_back(corners_left);
                _2d_points_right.push_back(corners_right);
                _3d_points.push_back(_3d_corners);
            }

        }

            cv::Mat cam_mat_left, cam_mat_right, dist_coef_left, dist_coef_right, R, T, E, F; 
            // float error_left = calibrate_camera(_2d_points_left, _3d_points, camera_size, cam_mat_left, dist_coef_left);
            // float error_right = calibrate_camera(_2d_points_right, _3d_points, camera_size, cam_mat_right, dist_coef_right);
            // std::cout << "Error de reproyeccion: " << error << std::endl;


            float error = cv::stereoCalibrate(_3d_points, _2d_points_left, _2d_points_right, cam_mat_left, cam_mat_right, dist_coef_left, dist_coef_right, camera_size, R, T, E, F, 0.0, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 60, 1e-6));
            // La R y la T que nos da la función son la matriz de rotacion y el vector de traslación respectivos a una caámara entre otra
            //E y F representan lo mismo pero de distinta manera. 
            //Dado un pixel en la cámara izquierda al multiplicar E por el pixel, obtendré los coeficientes de ax+b = c que representan la línea en la que caerá el punto en la cámara derecha
            //F es igual que E pero representando un punto en vez de en pixeles como una distancia entre el punto y el centro de la cámara ((A-Cx)/fx)
            //F es en el espacio real y E en el pixelar

            auto fs = cv::FileStorage();
            fs.open(output_fname, cv::FileStorage::WRITE);
            save_calibration_parameters(fs, camera_size, error, cam_mat_left, dist_coef_left, cam_mat_right, dist_coef_right, R, T, E, F);
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
