#pragma once
#include <opencv2/core.hpp>

namespace fsiv{

struct CParams
{
    cv::Size cam_size;
    cv::Size prj_size;
    cv::Mat cam_K;
    cv::Mat cam_D;
    cv::Mat cam_rvec;
    cv::Mat cam_tvec;
    cv::Mat prj_K;
    cv::Mat prj_D;
    cv::Mat prj_rvec;
    cv::Mat prj_tvec;
};

bool load_calibration_parameters_from_file(const std::string& fname, CParams& cparams);
bool save_calibration_parameters_to_file(const std::string& fname, const CParams& cparams);

} //namespace fsiv;
