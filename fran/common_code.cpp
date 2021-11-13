#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include "common_code.hpp"

std::vector<cv::Point3f>
fsiv_generate_3d_calibration_points(const cv::Size& board_size,
                                    float square_size)
{
    std::vector<cv::Point3f> ret_v;
    //TODO
    //Remenber: the first inner point has (1,1) in board coordinates. Thats why we start on 1
    // board_size contiene los puntos interiores de ancho y alto por eso ponemos <=
    for (int y=1; y<=board_size.height; y++){
        for (int x=1; x<=board_size.width; x++) {
            ret_v.push_back(cv::Point3f(x*square_size, y*square_size,0.0));
        }
    }

    //
    CV_Assert(ret_v.size()==board_size.width*board_size.height);
    return ret_v;
}


bool
fsiv_find_chessboard_corners(const cv::Mat& img, const cv::Size &board_size,
                             std::vector<cv::Point2f>& corner_points,
                             const char * wname)
{
    CV_Assert(img.type()==CV_8UC3);
    bool was_found = false;
    //TODO
    was_found = cv::findChessboardCorners(img, board_size, corner_points);
    if (was_found) {
        // Tengo que darle la imagen monocromo
        cv::Mat aux;
        cv::cvtColor(img, aux, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(aux, corner_points, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS,30,0.01)); // Refina uno por uno los puntos internos de los cuadrados del tablero
    }

    if (wname) {
        cv::Mat aux = img.clone();
        cv::drawChessboardCorners(aux, board_size, corner_points, was_found);
        cv::imshow(wname, aux);
        cv::waitKey(0);
    }

    //
    return was_found;
}

float
fsiv_calibrate_camera(const std::vector<std::vector<cv::Point2f>>& _2d_points,
                      const std::vector<std::vector<cv::Point3f>>& _3d_points,
                      const cv::Size &camera_size,
                      cv::Mat& camera_matrix,
                      cv::Mat& dist_coeffs,
                      std::vector<cv::Mat>* rvecs,
                      std::vector<cv::Mat>* tvecs)
{
    CV_Assert(_3d_points.size()>=2 && _3d_points.size()==_2d_points.size());
    float error=0.0;
    //TODO
    std::vector<cv::Mat> rvecs_;
    std::vector<cv::Mat> tvecs_;
    error = cv::calibrateCamera(_3d_points, _2d_points, camera_size, camera_matrix, dist_coeffs, rvecs_, tvecs_); // rvecs y tvecs son salidas

    // si el usuario me lo pide se lo cargo
    if (rvecs!=nullptr) // vector de rotacion
        *rvecs = rvecs_;
    if (tvecs!=nullptr) //vector de traslacion
        *tvecs = tvecs_;
    //
    CV_Assert(camera_matrix.rows==camera_matrix.cols &&
              camera_matrix.rows == 3 &&
              camera_matrix.type()==CV_64FC1);
    CV_Assert((dist_coeffs.rows*dist_coeffs.cols) == 5 &&
              dist_coeffs.type()==CV_64FC1);
    CV_Assert(rvecs==nullptr || rvecs->size()==_2d_points.size());
    CV_Assert(tvecs==nullptr || tvecs->size()==_2d_points.size());
    return error;
}

void fsiv_compute_camera_pose(const std::vector<cv::Point3f> &_3dpoints,
                              const std::vector<cv::Point2f> &_2dpoints,
                              const cv::Mat& camera_matrix,
                              const cv::Mat& dist_coeffs,
                              cv::Mat& rvec,
                              cv::Mat& tvec)
{
    CV_Assert(_3dpoints.size()>=4 && _3dpoints.size()==_2dpoints.size());
    //TODO
    cv::solvePnP(_3dpoints, _2dpoints, camera_matrix, dist_coeffs, rvec, tvec); // solo se estudia la pose de la homografia

    //
    CV_Assert(rvec.rows==3 && rvec.cols==1 && rvec.type()==CV_64FC1);
    CV_Assert(tvec.rows==3 && tvec.cols==1 && tvec.type()==CV_64FC1);
}

void
fsiv_draw_axes(cv::Mat& img,               
               const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
               const cv::Mat& rvec, const cv::Mat& tvec,
               const float size, const int line_width)
{
    //TODO
    std::vector<cv::Point3f> _3dpoints = {cv::Point3f(0,0,0), cv::Point3f(size,0,0), cv::Point3f(0,size,0), cv::Point3f(0,0,-size)};
    
    std::vector<cv::Point2f> _2dpoints;
    cv::projectPoints(_3dpoints,rvec,tvec,camera_matrix,dist_coeffs,_2dpoints);
    
    cv::line(img, _2dpoints[0], _2dpoints[1], cv::Scalar(0,0,255), line_width);
    cv::line(img, _2dpoints[0], _2dpoints[2], cv::Scalar(0,255,0), line_width);
    cv::line(img, _2dpoints[0], _2dpoints[3], cv::Scalar(255,0,0), line_width);
    //
}

void
fsiv_save_calibration_parameters(cv::FileStorage& fs,
                                const cv::Size & camera_size,
                                float error,
                                const cv::Mat& camera_matrix,
                                const cv::Mat& dist_coeffs,
                                 const cv::Mat& rvec,
                                 const cv::Mat& tvec)
{
    CV_Assert(fs.isOpened());
    CV_Assert(camera_matrix.type()==CV_64FC1 && camera_matrix.rows==3 && camera_matrix.cols==3);
    CV_Assert(dist_coeffs.type()==CV_64FC1 && dist_coeffs.rows==1 && dist_coeffs.cols==5);
    CV_Assert(rvec.type()==CV_64FC1 && rvec.rows==3 && rvec.cols==1);
    CV_Assert(tvec.type()==CV_64FC1 && tvec.rows==3 && tvec.cols==1);
    //TODO
    fs <<"image-width" << camera_size.width;
    fs << "image-height" << camera_size.height;
    fs << "error" << error;
    fs << "camera-matrix" << camera_matrix;
    fs << "distorsion-coefficients" << dist_coeffs;
    fs << "rvec" << rvec;
    fs << "tvec" << tvec;

    //
    CV_Assert(fs.isOpened());
    return;
}

void
fsiv_load_calibration_parameters(cv::FileStorage &fs,
                                 cv::Size &camera_size,
                                 float& error,
                                 cv::Mat& camera_matrix,
                                 cv::Mat& dist_coeffs,
                                 cv::Mat& rvec,
                                 cv::Mat& tvec)
{
    CV_Assert(fs.isOpened());
    //TODO
    fs ["image-width"] >> camera_size.width;
    fs ["image-height"] >> camera_size.height;
    fs ["error"] >> error;
    fs ["camera-matrix"] >> camera_matrix;
    fs ["distorsion-coefficients"] >> dist_coeffs;
    fs ["rvec"] >> rvec;
    fs ["tvec"] >> tvec;



    //
    CV_Assert(fs.isOpened());
    CV_Assert(camera_matrix.type()==CV_64FC1 && camera_matrix.rows==3 && camera_matrix.cols==3);
    CV_Assert(dist_coeffs.type()==CV_64FC1 && dist_coeffs.rows==1 && dist_coeffs.cols==5);
    CV_Assert(rvec.type()==CV_64FC1 && rvec.rows==3 && rvec.cols==1);
    CV_Assert(tvec.type()==CV_64FC1 && tvec.rows==3 && tvec.cols==1);
    return;
}

void
fsiv_undistort_image(const cv::Mat& input, cv::Mat& output,
                     const cv::Mat& camera_matrix,
                     const cv::Mat& dist_coeffs)
{
    //TODO
    //Hint: use cv::undistort.
    output = input.clone();

    //
}

void
fsiv_undistort_video_stream(cv::VideoCapture&input_stream,
                            cv::VideoWriter& output_stream,
                            const cv::Mat& camera_matrix,
                            const cv::Mat& dist_coeffs,
                            const int interp,
                            const char * input_wname,
                            const char * output_wname,
                            double fps)
{
    CV_Assert(input_stream.isOpened());
    CV_Assert(output_stream.isOpened());
    //TODO
    //Hint: to speed up, first compute the transformation maps
    //(one time only at the beginning using cv::initUndistortRectifyMap)
    // and then only remap (cv::remap) the input frame with the computed maps.

    //
    CV_Assert(input_stream.isOpened());
    CV_Assert(output_stream.isOpened());
}
