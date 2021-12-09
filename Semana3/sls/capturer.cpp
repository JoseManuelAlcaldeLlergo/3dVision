#include <iostream>
#include "capturer.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace fsiv {

Capturer::Capturer(int c_idx)
{
    auto dummy = cv::FileStorage();
    open(c_idx, dummy);
}

Capturer::Capturer(int c_idx, cv::FileStorage& cparams)
{
    open(c_idx, cparams);    
}

Capturer::~Capturer()
{
    cap_.release();
}

bool
Capturer::open(int c_idx, cv::FileStorage& cparams)
{
    gaussian_r_ = 1;
    show_wait_=1000;
    NUM_GRAB_PER_SHOT_=10;
    NUM_AVG_IMAGES_=1;
    if (cparams.isOpened())
    {
        auto node = cparams["SHOW_WAIT"];
        if (!node.empty())
            show_wait_ = int(node.real());
        node = cparams["NUM_GRAB_PER_SHOT"];
        if (!node.empty())
            NUM_GRAB_PER_SHOT_=int(node.real());
        node = cparams["NUM_AVG_IMAGES"];
        if (!node.empty())
            NUM_AVG_IMAGES_=int(node.real());
        node = cparams["GAUSSIAN_R"];
        if (!node.empty())
            gaussian_r_=int(node.real());
    }
    bool was_ok = true;
    if (cap_.open(c_idx))
    {
        cv::Mat img;
        if (cap_.read(img))
            img_size_ = img.size();
        else
        {
            cap_.release();
            was_ok = false;
        }
    }
    else
        was_ok = false;
    return was_ok;
}

bool
Capturer::is_opened() const
{    
    return cap_.isOpened();
}

static void
exposure_onChange(int value, void * user_data)
{
    cv::VideoCapture * cap = reinterpret_cast<cv::VideoCapture*>(user_data);
    double v = value/100.0;
    cap->set(cv::CAP_PROP_EXPOSURE, v);
    std::cout << "Camera exposure: "
              << cap->get(cv::CAP_PROP_EXPOSURE) << std::endl;
}

static void
brightness_onChange(int value, void * user_data)
{
    cv::VideoCapture * cap = reinterpret_cast<cv::VideoCapture*>(user_data);
    double v = value/100.0;
    cap->set(cv::CAP_PROP_BRIGHTNESS, v);
    std::cout << "Camera BRIGHTNESS: "
              << cap->get(cv::CAP_PROP_BRIGHTNESS) << std::endl;
}

static void
gain_onChange(int value, void * user_data)
{
    cv::VideoCapture * cap = reinterpret_cast<cv::VideoCapture*>(user_data);
    double v = value/100.0;
    cap->set(cv::CAP_PROP_GAIN, v);
    std::cout << "Camera GAIN: "
              << cap->get(cv::CAP_PROP_GAIN) << std::endl;
}

static void
autoexposure_onChange(int value, void * user_data)
{
    cv::VideoCapture * cap = reinterpret_cast<cv::VideoCapture*>(user_data);
    double v = value/100.0;
    cap->set(cv::CAP_PROP_AUTO_EXPOSURE, v);
    std::cout << "Camera AUTO_EXPOSURE: "
              << cap->get(cv::CAP_PROP_AUTO_EXPOSURE) << std::endl;
}

static void
resolution_onChange(int value, void * user_data)
{
    cv::VideoCapture * cap = reinterpret_cast<cv::VideoCapture*>(user_data);
    int x_res = int (640.0 + value/100.0 * (1920.0-640.0));
    int y_res = int (480.0 + value/100.0 * (1080.0-480.0));
    cap->set(cv::CAP_PROP_FRAME_WIDTH, x_res);
    cap->set(cv::CAP_PROP_FRAME_HEIGHT, y_res);
    std::cout << "Camera resolution: "
              << cap->get(cv::CAP_PROP_FRAME_WIDTH) << " x "
              << cap->get(cv::CAP_PROP_FRAME_HEIGHT)
              << std::endl;
}

bool Capturer::show_live_video(const char * wnd_name, int * key_)
{
    int key = -1;
    bool wasOk = true;
    cv::namedWindow(wnd_name, cv::WINDOW_GUI_NORMAL);
    std::cout << "Camera control parameters:" << std::endl;
    int exposure = int(cap_.get(cv::CAP_PROP_EXPOSURE)*100.0);
    std::cout << "Exposure = " << exposure << std::endl;
    int brightness = int(cap_.get(cv::CAP_PROP_BRIGHTNESS)*100.0);
    std::cout << "Brightness = " << brightness << std::endl;
    int gain = int(cap_.get(cv::CAP_PROP_GAIN)*100.0);
    std::cout << "Gain = " << gain << std::endl;
    int auto_exposure = int(cap_.get(cv::CAP_PROP_AUTO_EXPOSURE)*100.0);
    std::cout << "Autoexposure: "<< auto_exposure << std::endl;
    int resolution = int(cap_.get(cv::CAP_PROP_FRAME_WIDTH)-640.0)/(1920.0-640.0)*100.0;
    std::cout << "Resolution: " <<  cap_.get(cv::CAP_PROP_FRAME_WIDTH)
              << " x " << cap_.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
    cv::createTrackbar("Eposure", wnd_name, &exposure, 100, exposure_onChange, &cap_);
    cv::createTrackbar("Bright ", wnd_name, &brightness, 100, brightness_onChange, &cap_);
    cv::createTrackbar("Gain   ", wnd_name, &gain, 100, gain_onChange, &cap_);
    cv::createTrackbar("AutoExp", wnd_name, &auto_exposure, 100, autoexposure_onChange, &cap_);
    cv::createTrackbar("Resolut", wnd_name, &resolution, 100, resolution_onChange, &cap_);
    cv::Mat aux_img;
    while (wasOk && key == -1)
    {
        wasOk = cap_.read(aux_img);
        if (! wasOk)
        {
            std::cerr << "Error: could not capture an image."
                      << std::endl;
        }
        else
        {
            cv::imshow(wnd_name, aux_img);
            key = cv::waitKey(40); //25fps.
        }
    }
    cv::destroyWindow(wnd_name);
    if (key_)
        *key_ = key;
    return wasOk;
}


bool
Capturer::scan_pattern_sequence(Projector& prj,
                                std::shared_ptr<ScanningPatternSequence> &patterns)
{
    bool wasOk = true;
    for (size_t p=0; p<patterns->seq.size(); ++p)
    {
        int key = prj.project(patterns->seq[p], show_wait_);
        if (key == 27)
        {
            std::cerr << "Aborting scanning." << std::endl;
            wasOk = false;
        }
        else
        {
            cv::Mat img;
            wasOk = capture_image(img);
            if (wasOk)
                patterns->seq[p]=img;
        }
    }
    return wasOk;
}

/** @brief Captura una imagen de una cámara.
Params:
    img es la imagen donde guardar la captura
Returns:
    true si se realizó la captura correctamente.
*/
bool
Capturer::capture_image(cv::Mat& img_)
{
    bool was_ok = true;
    cv::Mat img, frame;
    int count_avg = NUM_AVG_IMAGES_;
    while (count_avg > 0 && was_ok)
    {
        int count_grab = NUM_GRAB_PER_SHOT_;
        while (was_ok && count_grab>0)
        {
            was_ok = cap_.grab();
            if (! was_ok)
                std::cerr << "Error: could not grab an image." << std::endl;
            else
                --count_grab;
        }
        if (was_ok)
        {
            was_ok  = cap_.retrieve(frame);
            if (was_ok)
            {
                if (NUM_AVG_IMAGES_>1)
                    frame.convertTo(frame, CV_32F);
                if (gaussian_r_>0)
                    cv::GaussianBlur(frame, frame, cv::Size(2*gaussian_r_+1,
                                                        2*gaussian_r_+1), 0.0);
                if (img.empty())
                    img = frame.clone();
                else                
                    img += frame;                
                --count_avg;
            }
            else
                std::cerr << "Error: could not retrieve an image." << std::endl;
        }
    }
    if (NUM_AVG_IMAGES_>1)
        img.convertTo(img_, CV_8U, 1.0/double(NUM_AVG_IMAGES_));
    else
        img_=img;
    return was_ok;
}

} //namespace
