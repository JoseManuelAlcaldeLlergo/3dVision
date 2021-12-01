#include "camera_parameters.hpp"
#include <opencv2/calib3d.hpp>

namespace fsiv {
CameraParameters::CameraParameters()
{
    _image_size=cv::Size(0,0);
    _camera_matrix=cv::Mat::eye(3, 3, CV_64FC1);
    _distoritions_coeffs=cv::Mat::zeros(1, 5, CV_64FC1);
    _rotation_vector=cv::Mat::zeros(3, 1, CV_64FC1);
    _translation_vector=cv::Mat::zeros(3, 1, CV_64FC1);
}

const cv::Size&
CameraParameters::image_size() const
{
    return _image_size;
}
const cv::Mat&
CameraParameters::camera_matrix() const
{
    return _camera_matrix;
}

const cv::Mat&
CameraParameters::distortion_coeffs () const
{
    return _distoritions_coeffs;
}

const cv::Mat&
CameraParameters::rotation_vector () const
{
    return _rotation_vector;
}

const cv::Mat&
CameraParameters::translation_vector () const
{
    return _translation_vector;
}

bool
CameraParameters::read_from_file(const std::string &pathname)
{
   bool retVal = true;
   //TODO
   //Hint: Use cv::FileStorage class to load the labels:
   //image-width, image-height,camera-matrix, distortion-coefficients,
   //tvec, rvec.
   cv::FileStorage input;
   if (input.open(pathname,cv::FileStorage::READ))
   {
       input["image-width"] >> _image_size.width;
       input["image-height"] >> _image_size.height;
       input["camera-matrix"] >> _camera_matrix;
       input["distortion-coefficients"] >>_distoritions_coeffs;
       auto node = input["translation-vector"];
       if (!node.isNone())
            node >>_translation_vector;
       node = input["tvec"];
       if (!node.isNone())
            node >>_translation_vector;
       node = input["rotation-matrix"];
       if (!node.isNone())
       {
           cv::Mat rotmat;
           node >> rotmat;
           cv::Rodrigues(rotmat,_rotation_vector);
       }
       node = input["rvec"];
       if (!node.isNone())
           node >> _rotation_vector;
   }
   else
       retVal=false;
   //
   return retVal;
}

} //namespace fsiv
