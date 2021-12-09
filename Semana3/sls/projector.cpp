#include "projector.hpp"
#include <opencv2/highgui.hpp>

namespace fsiv {

static const char * DEFAULT_PRJ_WNAME_="PROJECTOR";

Projector::Projector(int x_orig, int y_orig, const char* wname)
{
    x_orig_=x_orig;
    y_orig_=y_orig;
    wname_=wname;
    if (wname_==nullptr)
        wname_=DEFAULT_PRJ_WNAME_;
    switch_on();
}

Projector::~Projector()
{
    switch_off();
}

void
Projector::switch_on()
{
    cv::namedWindow(wname_, cv::WND_PROP_FULLSCREEN);
    cv::setWindowProperty(wname_, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::moveWindow(wname_, x_orig_, y_orig_);
}

void
Projector::switch_off()
{
    cv::destroyWindow(wname_);
}

int
Projector::project(const cv::Mat & pattern, int wait) const
{
    cv::imshow(wname_, pattern);
    int key = cv::waitKey(wait);
    return (key==-1) ? (key) : (key & 0xff);
}

} //namespace
