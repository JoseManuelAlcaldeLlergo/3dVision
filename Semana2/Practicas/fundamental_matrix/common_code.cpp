#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <sys/stat.h>
#include <fstream>
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
    __imshow("Fundamental",Mixed,cv::Size{1600,600});

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
            __imshow("Fundamental",Mixed,cv::Size{1600,600});
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



