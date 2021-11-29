#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

//Structure that contains the Stereo Pair Calbration information.
//This will be calculated using stereo_calibrate
struct CP{
   cv::Mat camera_matrix, dist_coefs;
};

struct OnMouseParams{
   cv::Mat img;
   std::string wname;
};


bool IsPathExist(const std::string &s);

CP readCameraParams(cv::FileStorage &fs);

void writeToPCD(std::string path,std::vector<cv::Point3f> points);

void showEpipolar(cv::Mat centralImage,cv::Mat otherImage,cv::Mat CamK,cv::Mat F);

cv::Mat removeDistortion(cv::Mat img, CP cam_params);

void __imshow(std::string wn,const cv::Mat &im,cv::Size ss);

void showEpipolar(cv::Mat centralImage,cv::Mat otherImage,cv::Mat CamK,cv::Mat F);

cv::Mat fundamental(cv::Mat im1,cv::Mat im2);

std::vector<cv::DMatch>  KpMatch( std::vector<cv::KeyPoint> keypoints_query ,cv::Mat descriptors_query, std::vector<cv::KeyPoint> keypoints_train, cv::Mat descriptors_train , cv::Mat im1, cv::Mat im2, std::vector<cv::KeyPoint> &keypoints_query_filtered,std::vector<cv::KeyPoint> &keypoints_train_filtered);


