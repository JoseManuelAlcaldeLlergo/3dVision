#pragma once
#include <opencv2/core.hpp>

namespace fsiv {


class Projector
{
public:
    Projector(int x_orig=0, int y_orig=0, const char* wname=nullptr);
    ~Projector();
    int project(const cv::Mat& pattern, int wait=1000) const;
    void switch_on();
    void switch_off();
private:
    int x_orig_;
    int y_orig_;
    const char * wname_;
    bool has_windows_; /** Hay un ventana ya creada?**/
};

} //namespace
