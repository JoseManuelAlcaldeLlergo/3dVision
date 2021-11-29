#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <sys/stat.h>
#include <fstream>
#include <iostream>
#include "common_code.hpp"

CP readCameraParams(cv::FileStorage &fs)
{
    CV_Assert(fs.isOpened());

    CP cam_params;
    
    fs ["camera_matrix"] >> cam_params.camera_matrix;
    fs ["distortion_coefficients"] >> cam_params.dist_coefs;

    return cam_params;
}

void writeToPCD(std::string path,std::vector<cv::Point3f> points){
   std::ofstream file(path,std::ios::binary);
   if(!file)throw std::runtime_error("Could not ope  file:"+path);
   file<<"# .PCD v.7 - Point Cloud Data file format"<<std::endl<<"VERSION .7"<<std::endl;
   file<<"FIELDS x y z "<<std::endl<<"SIZE 4 4 4 "<<std::endl;
   file<<"TYPE F F F "<<std::endl<<"COUNT 1 1 1 "<<std::endl;
   file<<"WIDTH "<<points.size()<<std::endl<<"HEIGHT 1"<<std::endl;
   file<<"VIEWPOINT 0 0 0 1 0 0 0"<<std::endl;
   file<<"POINTS "<<points.size()<< std::endl;
   file<<"DATA binary"<<std::endl;
   file.write((char*)&points[0],sizeof(cv::Point3f)*points.size() );
}


bool IsPathExist(const std::string &s)
{
  struct stat buffer;
  return (stat (s.c_str(), &buffer) == 0);
}



void __imshow(std::string wn,const cv::Mat &im,cv::Size ss)
{

    if( im.size()==ss) cv::imshow(wn,im);
    else{
        cv::Mat resized;
        cv::resize(im,resized,ss);
        cv::imshow(wn,resized);
    }
}

void showEpipolar(cv::Mat centralImage,cv::Mat otherImage,cv::Mat CamK,cv::Mat F){

    if(F.empty())return;

    struct CallBackInfo
    {
        cv::Mat imA,imB;
        cv::Mat CamK;
        cv::Mat F;
    };

    CallBackInfo *cbi=new CallBackInfo{centralImage,otherImage,CamK,F};

    cv::Mat Mixed(centralImage.rows,centralImage.cols*2,CV_8UC3);
    if( centralImage.channels()==1){
        cv::cvtColor(centralImage,Mixed.colRange(0,centralImage.cols),cv::COLOR_GRAY2BGR);
        cv::cvtColor(otherImage,Mixed.colRange(centralImage.cols,2*centralImage.cols ),cv::COLOR_GRAY2BGR);
    }
    else{
        centralImage.copyTo(Mixed.colRange(0,centralImage.cols));
        otherImage.copyTo(Mixed.colRange(centralImage.cols,2*centralImage.cols));
    }

     cv::namedWindow("Fundamental");
    __imshow("Fundamental",Mixed,cv::Size{800,300});

    cv::setMouseCallback("Fundamental",[](int event, int x, int y, int flags, void* _cbi)->void{
        if(x<800){
            CallBackInfo *cbi=(CallBackInfo*)_cbi;

            int sX=float(x)*float(cbi->imB.cols)/800.;
            int sY=float(y)*(float(cbi->imB.rows)/600.);

            //find epipolar line
            cv::Mat P= (cv::Mat_<double>(3,1) << sX, sY, 1);
            cv::Mat L=cbi->F*P;//epipolar line
            double a=L.at<double>(0,0);
            double b=L.at<double>(0,1);
            double c=L.at<double>(0,2);
            int x0=0;
            int y0=(a*x0+c)/-b;
            int x1=cbi->imB.cols;
            int y1=(a*x1+c)/-b;
            cv::Mat imBcopy;
            cv::cvtColor(cbi->imB,imBcopy,cv::COLOR_GRAY2BGR);
            cv::line(imBcopy,cv::Point(x0,y0),cv::Point(x1,y1),{0,0,244},2);

            cv::Mat Mixed(cbi->imA.rows,cbi->imA.cols*2,CV_8UC3);
            cv::cvtColor(cbi->imA,Mixed.colRange(0,cbi->imA.cols),cv::COLOR_GRAY2BGR);
            imBcopy.copyTo(Mixed.colRange(cbi->imA.cols,2*cbi->imA.cols ));

            cv::namedWindow("Fundamental");
            __imshow("Fundamental",Mixed,cv::Size{800,300});
        }else{


        }
    },cbi
    );
}


cv::Mat removeDistortion(cv::Mat img, CP cam_params){
    cv::Mat und_img;
    cv::undistort(img, und_img,cam_params.camera_matrix, cam_params.dist_coefs);

    return und_img;

}

std::vector<cv::DMatch> KpMatch( std::vector<cv::KeyPoint> keypoints_query ,cv::Mat descriptors_query, std::vector<cv::KeyPoint> keypoints_train, cv::Mat descriptors_train, cv::Mat im1, cv::Mat im2, std::vector<cv::KeyPoint> &keypoints_query_filtered,std::vector<cv::KeyPoint> &keypoints_train_filtered){
    
    std::vector< std::vector<cv::DMatch> > matches;
    std::vector<cv::DMatch>  filt_matches;
    cv::Mat filter_matches_image;
    
    auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->knnMatch(descriptors_query, descriptors_train, matches, 2);

    for(unsigned int i=0; i<matches.size();i++){
        // Probar con 3 octaves
        if(matches[i][0].distance < 80 &&
             abs(keypoints_query[matches[i][0].queryIdx].octave - keypoints_train[matches[i][0].trainIdx].octave) <= 1 &&
                abs(matches[i][0].distance - matches[i][1].distance) >= 0.8*matches[i][0].distance )     
        {
            // The current size of the filtered matches vector will define the index of te new match
            int current_size = static_cast<int>( keypoints_query_filtered.size());
            filt_matches.push_back(cv::DMatch(current_size, current_size, 0));
            keypoints_query_filtered.push_back(keypoints_query[matches[i][0].queryIdx]);
            keypoints_train_filtered.push_back(keypoints_train[matches[i][0].trainIdx]);
        }
        
    }
    
    // cv::drawMatches(im1, keypoints_query_filtered, im2, keypoints_train_filtered, filt_matches, filter_matches_image);

    // cv::namedWindow("Matches", CV_WINDOW_NORMAL);
    // cv::imshow("Matches", filter_matches_image);
    
    return filt_matches;
}

cv::Mat fundamental(cv::Mat im1,cv::Mat im2){
    cv::Mat F;
    std::vector<cv::KeyPoint> keypoints_query, keypoints_train,keypoints_query_filtered, keypoints_train_filtered, kp_query_filt_out, kp_train_filt_out;
    cv::Mat descriptors_query, descriptors_train;
    cv::Mat matches_image, filter_matches_image;

    auto Detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 1e-4f);
    Detector->detectAndCompute(im1, cv::Mat(), keypoints_query, descriptors_query);
    Detector->detectAndCompute(im2, cv::Mat(), keypoints_train, descriptors_train);

    std::vector<cv::DMatch> matches = KpMatch(keypoints_query, descriptors_query, keypoints_train, descriptors_train, im1, im2,keypoints_query_filtered, keypoints_train_filtered);
    std::vector<cv::DMatch> goodMatches;

    std::vector<cv::Point2f> points_query,points_train; //rellenar

    for(int i = 0; i < keypoints_query_filtered.size(); i++){
        points_query.push_back(keypoints_query_filtered[i].pt);
        points_train.push_back(keypoints_train_filtered[i].pt);
    }

    cv::drawMatches(im1, keypoints_query_filtered, im2, keypoints_train_filtered, matches, matches_image);

    cv::namedWindow("Matches", CV_WINDOW_NORMAL);
    cv::imshow("Matches", matches_image);

    cv::Mat inliers, filt_inliers;
    F = cv::findFundamentalMat(points_query,points_train,cv::FM_RANSAC,0.999,1.0,300,inliers);

    for(size_t i=0;i<inliers.total();i++){
       if( !inliers.ptr<uchar>(0)[i]){
        //ES OUTLIER
        }
        else{
            int current_size = static_cast<int>( kp_query_filt_out.size());
            goodMatches.push_back(cv::DMatch(current_size, current_size, 0));
            kp_query_filt_out.push_back(keypoints_query_filtered[matches[i].queryIdx]);
            kp_train_filt_out.push_back(keypoints_train_filtered[matches[i].trainIdx]);

        }
    }

    cv::drawMatches(im1, kp_query_filt_out, im2, kp_train_filt_out, goodMatches, filter_matches_image);

    cv::namedWindow("Good Matches", CV_WINDOW_NORMAL);
    cv::imshow("Good Matches", filter_matches_image);

    // std::vector<uchar>::const_iterator itIn= inliers.begin();
    // std::vector<cv::DMatch>::const_iterator   itM= matches.begin();
    // for ( ;itIn!= inliers.end(); ++itIn, ++itM)
    // {
    //     if (*itIn)
    //     {
    //         goodMatches.push_back(*itM);
    //     }
    // }


    return F;
}






