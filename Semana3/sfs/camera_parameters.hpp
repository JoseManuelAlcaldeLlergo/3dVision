#pragma once
#include <iostream>
#include <opencv2/core.hpp>


namespace fsiv
{

/**
 * @brief The CameraParameters class *
 */
class CameraParameters
{
public:
    CameraParameters();
    bool read_from_file(const std::string& pathname);
    const cv::Size& image_size() const;
    const cv::Mat& camera_matrix() const;
    const cv::Mat& distortion_coeffs () const;
    const cv::Mat& rotation_vector () const;
    cv::Mat rotation_matrix () const;
    const cv::Mat& translation_vector () const;

private:
    cv::Size _image_size;
    cv::Mat _camera_matrix;
    cv::Mat _distoritions_coeffs;
    cv::Mat _rotation_vector;
    cv::Mat _translation_vector;
};



} //namespacde fsiv
