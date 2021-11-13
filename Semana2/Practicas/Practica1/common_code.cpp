#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "common_code.hpp"

std::vector<cv::Point3f> generate_3d_calibration_points(const cv::Size& board_size,
                                    float square_size)
{
    std::vector<cv::Point3f> ret_v;
    //TODO

    // EMpiezan los bucles en 1 porque el primer punto interior es el (1,1)
    for (int y = 1; y<=board_size.height; y++){
        for(int x=1; x<=board_size.width; x++){
            ret_v.push_back(cv::Point3f(x*square_size, y*square_size, 0.0));//Z se pone a 0 porque el tablero es plano
        }

    }

    //
    CV_Assert(ret_v.size()==board_size.width*board_size.height);
    return ret_v;
}

bool find_chessboard_corners(const cv::Mat& img, const cv::Size &board_size,
                                  std::vector<cv::Point2f>& corner_points,
                                  const char * wname=nullptr){

    CV_Assert(img.type()==CV_8UC3);
    bool was_found = false;
    //TODO

    was_found = cv::findChessboardCorners(img, board_size, corner_points);

    if(was_found){
        cv::Mat aux;
        cv::cvtColor(img,aux,cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(aux, corner_points, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria( cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));
    }

    if(wname){
        cv::Mat aux = img.clone();
        cv::drawChessboardCorners(aux,board_size,corner_points, was_found);
        cv::imshow(wname, aux);
        cv::waitKey(0);
    }

    //
    return was_found;
}