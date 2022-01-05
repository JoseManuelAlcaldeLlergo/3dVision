#include "common_code.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

cv::Mat fsiv_color_rescaling(const cv::Mat &in, const cv::Scalar &from, const cv::Scalar &to)
{
    CV_Assert(in.type() == CV_8UC3);
    cv::Mat out;
    //TODO
    //Cuidado con dividir por cero.
    //Evita los bucles.

    //Por el aserto sabemos que la imagen tendrá 3 canales
    //Scalar es una clase de opencv que contiene 4 valores, uno por cada uno de los canales con los que puede trabajar opencv, aunque nosotros usemos 3
    cv::Scalar s = cv::Scalar(to[0] / from[0], to[1] / from[1], to[2] / from[2]);
    out = in.mul(s);

    //
    CV_Assert(out.type() == in.type());
    CV_Assert(out.rows == in.rows && out.cols == in.cols);
    return out;
}

cv::Mat fsiv_wp_color_balance(cv::Mat const &in)
{
    CV_Assert(in.type() == CV_8UC3);
    cv::Mat out;
    //TODO
    //Sugerencia: utiliza el espacio de color GRAY para
    //saber la ilumimancia de un pixel.

    // Convertimos la imagen de RGB a gris, porque solo nos interesa la luminancia
    cv::Mat gray;
    // Asumimos que la imagen de entrada está en BGR, no RGB
    cv::cvtColor(in, gray, cv::COLOR_BGR2GRAY);

    // Buscamos la posición del primer valor máximo de la matriz
    cv::Point maxLoc;
    // Todo lo que está a 0 indica que son valores que no me interesa conocer
    cv::minMaxLoc(gray, 0, 0, 0, &maxLoc);
    // Cojo el valor rgb correspondiente al punto más blanco
    cv::Scalar max_c = in.at<cv::Vec3b>(maxLoc);

    out = fsiv_color_rescaling(in, max_c, cv::Scalar::all(255.0));

    //
    CV_Assert(out.type() == in.type());
    CV_Assert(out.rows == in.rows && out.cols == in.cols);
    return out;
}

cv::Mat fsiv_gw_color_balance(cv::Mat const &in)
{
    CV_Assert(in.type() == CV_8UC3);
    cv::Mat out;
    //TODO

    //En este caso vamos a buscar el tono promedio de la imagen, y a reescalar para que sea 128 (256/2)
    cv::Scalar mean_c = cv::mean(in);
    out = fsiv_color_rescaling(in, mean_c, cv::Scalar::all(128.0));

    //
    CV_Assert(out.type() == in.type());
    CV_Assert(out.rows == in.rows && out.cols == in.cols);
    return out;
}

cv::Mat fsiv_color_balance(cv::Mat const &in, float p)
{
    CV_Assert(in.type() == CV_8UC3);
    CV_Assert(0.0f < p && p < 100.0f);
    cv::Mat out;
    //TODO
    //Sugerencia: utiliza el espacio de color GRAY para
    //saber la ilumimancia de un pixel.

    // Convertimos la imagen de RGB a gris, porque solo nos interesa la luminancia
    cv::Mat gray;
    std::vector<float> hist(256, 0);

    // Asumimos que la imagen de entrada está en BGR, no RGB
    cv::cvtColor(in, gray, cv::COLOR_BGR2GRAY);

    for (int y = 0; y < gray.rows; y++)
    {
        for (int x = 0; x < gray.cols; x++)
        {
            hist[gray.at<uchar>(y, x)] += 1;
        }
    }

    double total_points = gray.rows * gray.cols;
    // std::cout<<"Total points: "<<total_points<<std::endl;

    double p_points = total_points * (1 - (p / 100));
    // std::cout<<"Buscamos los : "<<p_points<<" puntos más brillantes"<<std::endl;

    double suma = 0.0;
    int tono_divisor_brillantes;

    // Alcanzamos el tono acorde al percentil establecido
    for (tono_divisor_brillantes = 255; suma < p_points; tono_divisor_brillantes--)
    {
        suma += hist[tono_divisor_brillantes];
    }

    // std::cout << "El tono que divide los puntos más brillantes es el " << tono_divisor_brillantes << std::endl;

    cv::Mat mask = (gray >= tono_divisor_brillantes) / 255;

    cv::Scalar mean_c = cv::mean(in, mask);

    out = fsiv_color_rescaling(in, mean_c, cv::Scalar::all(255.0));

    //Print hist
    // for (int it=0; it<hist_len; it++){
    //     std::cout<<"AQUI i="<<it<<" "<<hist[it]<<std::endl;
    // }

    // Luego con eso calculo en la imagen qué valores son más brillantes que ese tono y creo una máscara para obtener la media de esos valores
    // Ahora calculo la medi a con la máscara obtenida

    //
    CV_Assert(out.type() == in.type());
    CV_Assert(out.rows == in.rows && out.cols == in.cols);
    return out;
}
