#include "cparams.hpp"

namespace fsiv {

bool
load_calibration_parameters_from_file(const std::string& fname, CParams& cparams)
{
    bool was_ok = true;
    auto file = cv::FileStorage();
    was_ok = file.open(fname, cv::FileStorage::READ);
    if (was_ok)
    {
        int width, height;
        file["prj-width"]>> width;
        file["prj-height"]>> height;
        cparams.prj_size = cv::Size(width, height);
        file["prj-camera-matrix"] >> cparams.prj_K;
        file["prj-distortion-coefficients"]>>cparams.prj_D;
        file["prj-rotation-vector"]>>cparams.prj_rvec;
        file["prj-translation-vector"]>>cparams.prj_tvec;
        file["cam-width"]>>width;
        file["cam-height"]>>height;
        cparams.cam_size=cv::Size(width, height);
        file["cam-camera-matrix"]>>cparams.cam_K;
        file["cam-distortion-coefficients"]>>cparams.cam_D;
        file["cam-rotation-vector"]>>cparams.cam_rvec;
        file["cam-translation-vector"]>>cparams.cam_tvec;
    }
    return was_ok;
}


bool
save_calibration_parameters_to_file(const std::string& fname, const CParams& cparams)
{
    bool was_ok = true;
    auto file = cv::FileStorage();
    was_ok = file.open(fname, cv::FileStorage::WRITE);
    if (was_ok)
    {
        file<<"prj-width"<<cparams.prj_size.width;
        file<<"prj-height"<<cparams.prj_size.height;
        file<<"prj-camera-matrix"<< cparams.prj_K;
        file<<"prj-distortion-coefficients"<<cparams.prj_D;
        file<<"prj-rotation-vector"<<cparams.prj_rvec;
        file<<"prj-translation-vector"<<cparams.prj_tvec;
        file<<"cam-width"<<cparams.cam_size.width;
        file<<"cam-height"<<cparams.cam_size.height;
        file<<"cam-camera-matrix"<<cparams.cam_K;
        file<<"cam-distortion-coefficients"<<cparams.cam_D;
        file<<"cam-rotation-vector"<<cparams.cam_rvec;
        file<<"cam-translation-vector"<<cparams.cam_tvec;
    }
    return was_ok;
}

} //namespace fsvi
