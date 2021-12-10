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
    "{verbose        |      | activate verbose mode.}"
    "{i intrinsics   |      | Calibrate only extrinsics parameters. Using intrinsics from given file (-i=intr-file).}"
    "{s size         |<none>| square size.}"
    "{r rows         |<none>| number of board's rows.}"
    "{c cols         |<none>| number of board's cols.}"
    "{v video        |<none>| uses a video instead of a set of images.}"
    "{@output        |<none>| filename for output intrinsics file.}"
    "{@input1        |<none>| first board's view.}"
    "{@input2        |      | second board's view.}"
    "{@inputn        |      | ... n-idx board's view.}"
    ;

int
main (int argc, char* const* argv)
{
    int retCode=EXIT_SUCCESS;

    try {        
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Calibrate the intrinsics parameters of a camera.");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        float square_size = parser.get<float>("s");
        int rows = parser.get<int>("r");
        int cols = parser.get<int>("c");
        bool verbose = parser.has("verbose");
        std::string output_fname = parser.get<cv::String>("@output");
        if (!parser.check())
        {
            parser.printErrors();
            return EXIT_FAILURE;
        }

        //Get the inputs.
        //find the second argument without '-' at begin.
        int input = 1;
        bool found = false;
        while (input<argc && !found)
            found = argv[input++][0] != '-';
        //CV_Assert(input<argc);
        std::vector<std::string> input_fnames;        
        for (; input<argc; ++input)
            input_fnames.push_back(std::string(argv[input]));




        if (parser.has("i"))
        {
            //TODO
            //Make extrinsic calibration.
            //Remenber: only one view is needed.





            //
            if (verbose)
            {
                //TODO
                //Show WCS axis.


                //
            }
        }
        else
        {
            //TODO
            //Make an intrisic calibration.
            //Remember: For each view (at least two) you must find the
            //chessboard to get the 3D -> 2D matches.
            auto board_Size = cv::Size(cols-1,rows-1);
            std::string wname = "TABLERO";
            std::vector<std::vector<cv::Point2f>> _2dpoints;
            std::vector<std::vector<cv::Point3f>> _3dpoints;
            std::vector<cv::Point3f> _3d_corners = fsiv_generate_3d_calibration_points(board_Size, square_size);

            cv::Size camera_size;
            std::vector<cv::Mat> tvecs;
            std::vector<cv::Mat> rvecs;

            if(!parser.has("video")){

                for(size_t v; v<input_fnames.size(); v++){
                    cv::Mat img = cv::imread(input_fnames[v]);
                    camera_size=cv::Size(img.cols,img.rows);
                    //comprobar si se puede cargar
                    //comprobar si todas las imagene son del mismo tamaño

                    std::vector<cv::Point2f> corners;
                    if (fsiv_find_chessboard_corners(img, board_Size, corners, wname.c_str())){
                        _2dpoints.push_back(corners);
                        _3dpoints.push_back(_3d_corners);
                    }

                }

                cv::Mat cameraMatrix, dist_coeffs;
                float error = fsiv_calibrate_camera(_2dpoints,_3dpoints, camera_size, cameraMatrix, dist_coeffs, &rvecs, &tvecs);
                std::cout<<"Error de reproyección: "<<error<<std::endl;
                

                auto fs = cv::FileStorage();
                fs.open(output_fname, cv::FileStorage::WRITE);
                // (fs,camera_size,);

                fsiv_save_calibration_parameters(fs,camera_size,error,cameraMatrix, dist_coeffs, rvecs[0], tvecs[0]);
                //

                if (verbose)
                {
                    //TODO
                    //Show WCS axis on each pattern view.
                    for(size_t v=0; v<input_fnames.size(); v++){
                        cv::Mat img = cv::imread(input_fnames[v]);
                        fsiv_draw_axes(img,cameraMatrix,dist_coeffs,rvecs[v],tvecs[v],square_size, 3);
                        cv::imshow("EJES",img);
                        cv::waitKey(0);
                    }

                    //
                }
            }
            //Video
            else{
                cv::VideoCapture capture;
                capture.open(cv::samples::findFileOrKeep(parser.get<std::string>("video")));

                while(true){
                    cv::Mat view, viewGray;
                    bool blink = false;

                    if( capture.isOpened() )
                    {
                        cv::Mat view0;
                        capture >> view0;
                        view0.copyTo(view);
                    }

                    if(view.empty())
                    {
                        
                        break;//Fin while
                    }

                    camera_size=cv::Size(view.cols,view.rows);
                    //comprobar si se puede cargar
                    //comprobar si todas las imagene son del mismo tamaño

                    std::vector<cv::Point2f> corners;
                    if (fsiv_find_chessboard_corners(view, board_Size, corners, wname.c_str())){
                        _2dpoints.push_back(corners);
                        _3dpoints.push_back(_3d_corners);
                    }
                }
            }
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
