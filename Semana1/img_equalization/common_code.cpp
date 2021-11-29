#include <iostream>
#include "common_code.hpp"

cv::Mat
fsiv_compute_histogram(const cv::Mat &in, cv::Mat &hist)
{
    CV_Assert(in.type() == CV_8UC1);
    CV_Assert(hist.empty() || (hist.type() == CV_32FC1 &&
                               hist.rows == 256 && hist.cols == 1));
    //TODO
    //Tienes dos alternativas:
    //1- Implementar un recorrido por la imagen y calcular el histograma.
    //2- Usar la función cv::calcHist.
    //Sugerencia: implementa las dos para comparar.


    cv::Mat my_hist(256, 1, CV_32F, 0.0);
    for (int y = 0; y < in.rows; y++)
    {
        for (int x = 0; x < in.cols; x++)
        {
            my_hist.at<float>(0, in.at<uchar>(y, x)) += 1;
        }
    }

    hist = my_hist;


    CV_Assert(hist.type() == CV_32FC1);
    CV_Assert(hist.rows == 256 && hist.cols == 1);
    return hist;
}

void fsiv_normalize_histogram(cv::Mat &hist)
{
    CV_Assert(hist.type() == CV_32FC1);
    CV_Assert(hist.rows == 256 && hist.cols == 1);

    //TODO


    normalize(hist, hist, 1, 0, cv::NORM_L1, -1, cv::Mat());

    //
    CV_Assert(hist.type() == CV_32FC1);
    CV_Assert(hist.rows == 256 && hist.cols == 1);
}

void fsiv_accumulate_histogram(cv::Mat &hist)
{
    CV_Assert(hist.type() == CV_32FC1);
    CV_Assert(hist.rows == 256 && hist.cols == 1);

    //TODO
    for (int i = 0; i < 256; i++)
    {
        hist.at<float>(0, i) = hist.at<float>(0, i - 1) + hist.at<float>(i);
        //cout<<"histogramNC["<<i<<"]= "<<histogramN[i]<<endl;
    }

    //
}

cv::Mat
fsiv_create_equalization_lookup_table(const cv::Mat &hist,
                                      bool hold_median)
{
    CV_Assert(hist.type() == CV_32FC1);
    CV_Assert(hist.rows == 256 && hist.cols == 1);
    cv::Mat lkt;
    //TODO
    //Usa las funciones fsiv_normalize_histogram y fsiv_accumulate_histogram
    //para construir la tabla.
    lkt = hist.clone();
    fsiv_normalize_histogram(lkt);
    // std::cout<<lkt<<std::endl;
    fsiv_accumulate_histogram(lkt);
    // Como tenemos valores entre 0 y 1, hay que volver a la escala 0 255
    lkt = lkt * 255;
    // std::cout<<lkt<<std::endl;
    lkt.convertTo(lkt, CV_8UC1);
    // std::cout<<lkt<<std::endl;

    //

    CV_Assert(lkt.type() == CV_8UC1);
    CV_Assert(lkt.rows == 256 && lkt.cols == 1);
    return lkt;
}

cv::Mat
fsiv_apply_lookup_table(const cv::Mat &in, const cv::Mat &lkt,
                        cv::Mat &out)
{
    CV_Assert(in.type() == CV_8UC1);
    CV_Assert(lkt.type() == CV_8UC1);
    CV_Assert(lkt.rows == 256 && lkt.cols == 1);
    CV_Assert(out.empty() || (out.type() == CV_8UC1 &&
                              out.rows == in.rows && out.cols == in.cols));

    //TODO
    if (out.empty())
    {
        out = cv::Mat::zeros(in.size(), in.type());
    }
    for (int y = 0; y < in.rows; y++)
    {
        for (int x = 0; x < in.cols; x++)
        {
            out.at<uchar>(y, x) = lkt.at<uchar>(0, in.at<uchar>(y, x));
        }
    }

    //
    CV_Assert(out.rows == in.rows && out.cols == in.cols && out.type() == in.type());
    return out;
}

cv::Mat
fsiv_image_equalization(const cv::Mat &in, cv::Mat &out,
                        bool hold_median, int radius)
{
    CV_Assert(in.type() == CV_8UC1);
    //TODO
    //Utiliza las funciones fsiv_compute_histogram,
    //fsiv_create_equalization_lookup_table y fsiv_apply_lookup_table
    //

    if (out.empty())
        out = in.clone();

    // Si el radio es 0, aunque el resultado sea el mismo el bucle lo separamos para agilizar el proceso
    if (radius == 0)
    {
        cv::Mat hist(256, 1, CV_32F, 0.0);
        fsiv_compute_histogram(in, hist);
        cv::Mat lkt = fsiv_create_equalization_lookup_table(hist);
        fsiv_apply_lookup_table(in, lkt, out);
    }
    else
    {
        for (int y = radius; y < in.rows - radius; y++)
        {
            for (int x = radius; x < in.cols - radius; x++)
            {
                cv::Mat hist(256, 1, CV_32F, 0.0);
                cv::Mat partial_in = in(cv::Rect(x - radius, y - radius, (2 * radius) + 1, (2 * radius) + 1));
                cv::Mat partial_out = cv::Mat::zeros(cv::Size(2 * radius + 1, 2 * radius + 1), CV_8UC1);

                fsiv_compute_histogram(partial_in, hist);
                cv::Mat lkt = fsiv_create_equalization_lookup_table(hist);
                fsiv_apply_lookup_table(partial_in, lkt, partial_out);

                // Se reemplaza solamente el pixel central
                out.at<uchar>(y,x) = partial_out.at<uchar>(radius,radius);
            }

        }
    }

    //
    CV_Assert(out.rows == in.rows && out.cols == in.cols && out.type() == in.type());
    return out;
}
