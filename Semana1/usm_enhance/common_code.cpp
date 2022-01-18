#include "common_code.hpp"

cv::Mat
fsiv_create_box_filter(const int r)
{
    CV_Assert(r>0);
    cv::Mat ret_v;
    //TODO
    ret_v = cv::Mat::ones(2*r+1,2*r+1,CV_32FC1);
    cv::normalize(ret_v,ret_v, 1.0, 0.0, cv::NORM_L1);

    //
    CV_Assert(ret_v.type()==CV_32FC1);
    CV_Assert(ret_v.rows==(2*r+1) && ret_v.rows==ret_v.cols);
    CV_Assert(std::abs(1.0-cv::sum(ret_v)[0])<1.0e-6);
    return ret_v;
}

cv::Mat
fsiv_create_gaussian_filter(const int r)
{
    CV_Assert(r>0);
    cv::Mat ret_v;
    //TODO: Remember 6*sigma is approx 99,73% of the distribution.
    float sigma = (2*r+1.0)/6.0;
    ret_v = cv::Mat(2*r+1.0, 2*r+1.0, CV_32FC1);

    for(int y = -r; y <= r; y++){
        for(int x = -r; x <= r; x++){
            double v = std::exp(-0.5*((x*x + y*y)/(sigma*sigma)));
            ret_v.at<float>(y+r, x+r) = v;
        }
    }

    cv::normalize(ret_v, ret_v, 1, 0, cv::NORM_L1);

    //
    CV_Assert(ret_v.type()==CV_32FC1);
    CV_Assert(ret_v.rows==(2*r+1) && ret_v.rows==ret_v.cols);
    CV_Assert(std::abs(1.0-cv::sum(ret_v)[0])<1.0e-6);
    return ret_v;
}

cv::Mat
fsiv_fill_expansion(cv::Mat const& in, const int r)
{
    CV_Assert(!in.empty());
    CV_Assert(r>0);
    cv::Mat ret_v;
    //TODO:
    //Use of cv::copyMakeBorder.
    //Hint you don't need use any for sentence.
    
    cv::copyMakeBorder(in, ret_v, r, r, r, r, cv::BORDER_CONSTANT);


    //
    CV_Assert(ret_v.type()==in.type());
    CV_Assert(ret_v.rows == in.rows+2*r);
    CV_Assert(ret_v.cols == in.cols+2*r);
    return ret_v;
}

cv::Mat
fsiv_circular_expansion(cv::Mat const& in, const int r)
{
    CV_Assert(!in.empty());
    CV_Assert(r>0);
    cv::Mat ret_v;
    //TODO
    //Use of cv::copyMakeBorder.
    //Hint you don't need use any "for" sentence, only 9 copyTo from "in"
    // rois to "ret_v" rois.

    cv::copyMakeBorder(in, ret_v, r, r, r, r, cv::BORDER_WRAP);

    //
    CV_Assert(ret_v.type()==in.type());
    CV_Assert(ret_v.rows == in.rows+2*r);
    CV_Assert(ret_v.cols == in.cols+2*r);
    CV_Assert(!(in.type()==CV_8UC1) || ret_v.at<uchar>(0,0)==in.at<uchar>(in.rows-r, in.cols-r));
    CV_Assert(!(in.type()==CV_8UC1) || ret_v.at<uchar>(0,ret_v.cols/2)==in.at<uchar>(in.rows-r, in.cols/2));
    CV_Assert(!(in.type()==CV_8UC1) || ret_v.at<uchar>(0,ret_v.cols-1)==in.at<uchar>(in.rows-r, r-1));
    CV_Assert(!(in.type()==CV_8UC1) || ret_v.at<uchar>(ret_v.rows/2,0)==in.at<uchar>(in.rows/2, in.cols-r));
    CV_Assert(!(in.type()==CV_8UC1) || ret_v.at<uchar>(ret_v.rows/2,ret_v.cols/2)==in.at<uchar>(in.rows/2, in.cols/2));
    CV_Assert(!(in.type()==CV_8UC1) || ret_v.at<uchar>(ret_v.rows-1,0)==in.at<uchar>(r-1, in.cols-r));
    CV_Assert(!(in.type()==CV_8UC1) || ret_v.at<uchar>(ret_v.rows-1,ret_v.cols/2)==in.at<uchar>(r-1, in.cols/2));
    CV_Assert(!(in.type()==CV_8UC1) || ret_v.at<uchar>(ret_v.rows-1,ret_v.cols-1)==in.at<uchar>(r-1, r-1));
    return ret_v;
}

cv::Mat
fsiv_filter2D(cv::Mat const& in, cv::Mat const& filter)
{
    CV_Assert(!in.empty() && !filter.empty());
    CV_Assert(in.type()==CV_32FC1 && filter.type()==CV_32FC1);
    cv::Mat ret_v;
    //TODO
    
    int r = filter.rows/2;
    ret_v = cv::Mat(in.rows-2*r,in.cols-2*r, CV_32FC1);
    for(int y=0; y<ret_v.rows; y++){
        for(int x=0; x<ret_v.cols; x++){
            ret_v.at<float>(y,x) = cv::sum(filter.mul(
                                                        in(cv::Rect(x,y,filter.cols,filter.rows))))[0]; //Submatriz de la imagen que corresponde con el rectangulo del filtro
        }
    }

    //
    CV_Assert(ret_v.type()==CV_32FC1);
    CV_Assert(ret_v.rows==in.rows-2*(filter.rows/2));
    CV_Assert(ret_v.cols==in.cols-2*(filter.cols/2));
    return ret_v;
}

cv::Mat
fsiv_combine_images(const cv::Mat src1, const cv::Mat src2,
                    double a, double b)
{
    CV_Assert(src1.type()==src2.type());
    CV_Assert(src1.rows==src2.rows);
    CV_Assert(src1.cols==src2.cols);
    cv::Mat ret_v;
    //TODO

    ret_v = a*src1 + b*src2;

    //
    CV_Assert(ret_v.type()==src2.type());
    CV_Assert(ret_v.rows==src2.rows);
    CV_Assert(ret_v.cols==src2.cols);
    return ret_v;
}

cv::Mat
fsiv_usm_enhance(cv::Mat  const& in, double g, int r,
                 int filter_type, bool circular, cv::Mat *unsharp_mask)
{
    CV_Assert(!in.empty());
    CV_Assert(in.type()==CV_32FC1);
    CV_Assert(r>0);
    CV_Assert(filter_type>=0 && filter_type<=1);
    CV_Assert(g>=0.0);
    cv::Mat ret_v;
    //TODO
    //Hint: use your own functions fsiv_xxxx

    cv::Mat in_exp;
    if(!circular){
        in_exp = fsiv_fill_expansion(in,r);
    }
    else{
        in_exp = fsiv_circular_expansion(in,r);
    }

    cv::Mat filter;

    if(filter_type == 0){
        filter = fsiv_create_box_filter(r);
    }
    else{
        filter = fsiv_create_gaussian_filter(r);
    }

    cv::Mat unsharp_mask_ = fsiv_filter2D(in_exp, filter);

    if(unsharp_mask != nullptr){
        *unsharp_mask = unsharp_mask_;
    }

    ret_v = fsiv_combine_images(in, unsharp_mask_, (1.0+g),-g);

    //
    CV_Assert(ret_v.rows==in.rows);
    CV_Assert(ret_v.cols==in.cols);
    CV_Assert(ret_v.type()==CV_32FC1);
    return ret_v;
}
