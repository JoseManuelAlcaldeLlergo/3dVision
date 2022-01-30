#include <iostream>
#include <opencv2/calib3d.hpp>
#include "triangulation.hpp"

namespace fsiv
{

    cv::Mat
    compute_line_plane_triangulation(cv::Mat const &p_codes, int axis,
                                     CParams const &cparams, const cv::Mat &mask_)
    {
        CV_Assert(p_codes.type() == CV_16SC1);
        CV_Assert(mask_.empty() ||
                  (mask_.size() == p_codes.size() && mask_.type() == CV_8UC1));
        cv::Mat mask = mask_;
        if (mask.empty())
            mask = cv::Mat(p_codes.size(), CV_8UC1, 255.0);

        //Matriz con el resultado.
        cv::Mat XYZ = cv::Mat::zeros(p_codes.size(), CV_64FC3);
        //Recuerda para acceder a los valores de la matriz usaremos el tipo
        //cv::Vec3d -> XYZ.at<cv::Vec3d>(y, x)

        //Recuerda:
        //Si axis==1 los valores p_codes, codifican coordenada x del proyector
        //mientras que si axis==0 los valores codifican la coordenada y del proyector.
        //
        //Utiliza el tipo cv::int16_t para leer los valores de la matriz p_codes.
        //Utiliza usa CV_64F (double) para trabajar con valores reales.
        //
        //Revisa los atributos de la clase CParams para obtener los parámetros
        //intrínsecos/extrínsecos de la cámara y del proyector.
        //
        //Utiliza la máscara "mask" para calcular los valores XYZ sólo de los
        //pixeles activos.

        //TODO
        //
        //Sugerencia: usa los métodos cv::Mat::t() para obtener la matriz traspuesta,
        //cv::Mat::inv() para obtener la matriz inversa, cv::Mat::dot() para calcular
        //el producto escalar y el operador "<mat> * <mat>" para calcular el producto
        //de matrices.
        //
        //Sugerencia: cv::Rodrigues() para pasar de vector de rotación a
        //matriz de rotación.
        //
        //Para cada pixel <y,x> activo en la máscara:
        //- Obtener la coordenada x/y del proyector codificada de la
        //  matriz p_codes (cv::int16_t).
        //- Calcular la recta (WCS) que contiene el centro proyectivo de la cámara y
        //  punto <x,y> de la cámara.
        //- Calcular el plano (WCS) que contiene el centro proyectivo del proyector y
        //  la recta (vertical (axis==0)|horizontal(axis==1) con coordenada x/y del
        //  proyector por p_codes <y,x> (cv::int16_t)
        //- Calcular el punto intersección de la recta y el plano y almacenar sus
        //  coordenadas WCS  <X,Y,Z> en la matriz XYZ<y,x> usando el tipo cv::Vec3d().

        cv::Mat R_proj, R_cam;

        //Obtenemos las matrices de rotacion a partir de los vectores
        cv::Rodrigues(cparams.prj_rvec, R_proj);
        cv::Rodrigues(cparams.cam_rvec, R_cam);

        //Puntos de la camara(q_l) y el proyector (q_p). Seguiremos la nomenclatura de las diapositivas
        cv::Mat q_l = -R_cam.t() * cparams.cam_tvec;
        cv::Mat q_p = -R_proj.t() * cparams.prj_tvec;

        // La resta del numerador de lambada va a ser siempre el mismo por lo que evitamos calcularlo varias veces
        cv::Mat resta_num_lambda = q_p - q_l;

        for (size_t y = 0; y < p_codes.rows; y++)
        {
            for (size_t x = 0; x < p_codes.cols; x++)
            {
                //Para cada pixel <y,x> activo en la máscara:
                if (mask.at<uchar>(y, x) != 0)
                {

                    // Inicializamos el punto u con el pixel que toca
                    cv::Mat u(3, 1, CV_64F);
                    u.at<double>(0, 0) = x;
                    u.at<double>(1, 0) = y;
                    u.at<double>(2, 0) = 1.0;                   

                    //- Obtener la coordenada x/y del proyector codificada de la
                    //  matriz p_codes (cv::int16_t).
                    auto point = p_codes.at<cv::int16_t>(y, x);

                    //Calculamos la recta l y n teninedo en cuenta los axis si son verticales y horizontales
                    cv::Mat l = cv::Mat::ones(3, 1, CV_64F);

                    //Recta normal a l
                    cv::Mat n(3, 1, CV_64FC1);
                    
                    if (axis == 1)
                    {
                        l.at<double>(0, 0) = point;
                        l.at<double>(1, 0) = 0.0;
                        l.at<double>(2, 0) = 1.0;

                        //Pasamos la recta a unidades del mundo real
                        l = cparams.prj_K.inv() * l;

                        //Rellenamos la recta normal n
                        n.at<double>(0, 0) = 1.0;
                        n.at<double>(1, 0) = 0.0;
                        n.at<double>(2, 0) = -l.at<double>(0, 0);
                    }
                    else
                    {
                        l.at<double>(0, 0) = 0.0;
                        l.at<double>(1, 0) = point;
                        l.at<double>(2, 0) = 1.0;

                        //Pasamos la recta a unidades del mundo real
                        l = cparams.prj_K.inv() * l;

                        //Rellenamos la recta normal n
                        n.at<double>(0, 0) = 0.0;
                        n.at<double>(1, 0) = 1.0;
                        n.at<double>(2, 0) = -l.at<double>(1, 0);
                    }
                    //Convertimos la recta normal a unidades del mundo real
                    n = R_proj.t() * n;

                    //  Calculamos v
                    cv::Mat v = R_cam.t() * cparams.cam_K.inv() * u;

                    // Calculamos lambda
                    double lambda = n.dot(resta_num_lambda) / v.dot(n);

                    XYZ.at<cv::Vec3d>(y, x) = cv::Mat(q_l + lambda * v);
                }
            }
        }

        //
        CV_Assert(XYZ.size() == p_codes.size() && XYZ.type() == CV_64FC3);
        return XYZ;
    }

    cv::Mat
    compute_line_line_triangulation(cv::Mat const &x_codes,
                                    cv::Mat const &y_codes,
                                    CParams const &cparams,
                                    const cv::Mat &mask_)
    {
        CV_Assert(x_codes.type() == CV_16SC1);
        CV_Assert(y_codes.type() == CV_16SC1);
        CV_Assert(x_codes.size() == y_codes.size());
        CV_Assert(mask_.empty() ||
                  (mask_.size() == x_codes.size() && mask_.type() == CV_8UC1));
        cv::Mat mask = mask_;
        if (mask.empty())
            mask = cv::Mat(x_codes.size(), CV_8UC1, 255.0);

        //Matriz con el resultado.
        cv::Mat XYZ = cv::Mat::zeros(x_codes.size(), CV_64FC3);
        //Recuerda para acceder a los valores de la matriz usaremos el tipo
        //cv::Vec3d -> XYZ.at<cv::Vec3d>(y, x)

        //Recuerda:
        //Utiliza el tipo cv::int16_t para leer los valores de lsa matrices x_codes.
        // e y_codes.
        //
        //Revisa los atributos de la clase CParams para obtener los parámetros
        //intrínsecos/extrínsecos de la cámara y del proyector.
        //
        //Utiliza la máscara "mask" para calcular los valores XYZ sólo de los
        //pixeles activos.

        //TODO
        //Sugerencia: usa CV_64F (double) para trabajar con valores reales.
        //
        //Sugerencia: usa los métodos cv::Mat::t() para obtener la matriz traspuesta,
        //cv::Mat::inv() para obtener la matriz inversa, cv::Mat::dot() para calcular
        //el producto escalar y el operador "<mat> * <mat>" para calcular el producto
        //de matrices.
        //
        //Sugerencia: cv::Rodrigues() para pasar de vector de rotación a
        //matriz de rotación.
        //
        //Para cada pixel <x,y> activo en mask:
        //- Calcular la recta (WCS) que contiene el centro proyectivo de la cámara y
        //  punto <x,y>
        //- Calcular la recta (WCS) que contiene el centro proyectivo del proyector y
        //  el punto <x_codes<y,x>> , <y_codes<y,x>>.
        //- Calcular el punto más cercano a las dos rectas y almacenar las coordenadas
        //  <X,Y,Z> del punto en la matriz XYZ usando el tipo cv::Vec3d().

        //
        CV_Assert(XYZ.size() == x_codes.size() && XYZ.type() == CV_64FC3);
        return XYZ;
    }

    //VOLUNTARIA (no se usa en el codigo)Dice qie simplemente es aplicar la ecuacion de la recta en parametircas
    cv::Mat
    depth_map_to_XYZ(cv::Mat const &depth_m, CParams const &cparams, const cv::Mat &mask_)
    {
        CV_Assert(depth_m.type() == CV_64FC1);

        cv::Mat mask = mask_;
        if (mask.empty())
            mask = cv::Mat(depth_m.rows, depth_m.cols, CV_8UC1, 255.0);

        cv::Mat XYZ = cv::Mat::zeros(depth_m.rows, depth_m.cols, CV_64FC3);

        //Recuerda:
        //Revisa los atributos de la clase CParams para obtener los parámetros
        //intrínsecos/extrínsecos de la cámara y del proyector.
        //
        //Utiliza la máscara "mask" para calcular los valores XYZ sólo de los
        //pixeles activos.

        //TODO
        //Sugerencia: usa CV_64F (double) para trabajar con valores reales.
        //
        //Sugerencia: usa los métodos cv::Mat::t() para obtener la matriz traspuesta,
        //cv::Mat::inv() para obtener la matriz inversa, cv::Mat::dot() para calcular
        //el producto escalar y el operador "<mat> * <mat>" para calcular el producto
        //de matrices.
        //
        //Sugerencia: cv::Rodrigues() para pasar de vector de rotación a
        //matriz de rotación.
        //
        //Para cada pixel <x,y> activo de la matriz depth_m:
        //- Calcular la recta (WCS) que contiene el centro proyectivo de la cámara y
        //  pixel <x,y>
        //- Calcular el punto de la recta con parámetro lambda=depth_m<y,x>
        //  y almacenas las coordenadas <X,Y,Z> del punto en la matriz XYZ
        //  usando el tipo cv::Vec3d().

        //
        CV_Assert(XYZ.size() == depth_m.size() && XYZ.type() == CV_64FC3);
        return XYZ;
    }

    void save_XYZ_to_vrml(std::ostream &f,
                          cv::Mat const &XYZ,
                          cv::Mat const &img_color,
                          cv::Mat const &validity_mask_)
    {
        CV_Assert(XYZ.size() == img_color.size());
        CV_Assert(validity_mask_.empty() ||
                  (validity_mask_.size() == XYZ.size()));
        cv::Mat validity_mask = validity_mask_;
        if (validity_mask.empty())
            validity_mask = cv::Mat(XYZ.size(), CV_8UC1, 255);
        if (f)
        {
            f << "#VRML V2.0 utf8\n";
            f << "Transform\n";
            f << "{\n";
            f << "  children\n";
            f << "  Shape\n";
            f << "  {\n";
            f << "    appearance Appearance\n";
            f << "    {\n";
            f << "      material Material\n";
            f << "      {\n";
            f << "       emissiveColor 1.0 1.0 1.0\n";
            f << "      }\n";
            f << "    }\n";
            f << "    geometry PointSet\n";
            f << "    {\n";

            f << "      coord Coordinate\n";
            f << "      {\n";
            f << "        point\n";
            f << "          [\n";
            for (int r = 0; r < XYZ.rows; ++r)
                for (int c = 0; c < XYZ.cols; ++c)
                {
                    if (validity_mask.at<uchar>(r, c))
                    {
                        f << XYZ.at<cv::Vec3d>(r, c)[0] << ' '
                          << XYZ.at<cv::Vec3d>(r, c)[1] << ' '
                          << XYZ.at<cv::Vec3d>(r, c)[2] << '\n';
                    }
                }
            f << "          ] }\n";

            f << "      color Color { color [\n";

            for (int r = 0; r < img_color.rows; ++r)
                for (int c = 0; c < img_color.cols; ++c)
                    if (validity_mask.at<uchar>(r, c))
                        f << img_color.at<cv::Vec3b>(r, c)[2] / 255.0 << ' '
                          << img_color.at<cv::Vec3b>(r, c)[1] / 255.0 << ' '
                          << img_color.at<cv::Vec3b>(r, c)[0] / 255.0 << '\n';

            f << "         ]\n";
            f << "      }\n";
            f << "    }\n";
            f << "  }\n";
            f << "}\n";
        }
    }

    /** @brief Genera una máscara indicando que valores de un mapa de profundidad son válidos.
 *
 * Params:
    XYZ son las coordenadas de la nueve de puntos.
    x_range, y_range, z_range indican el rango de valores admisibles para cada eje.
    min_luma_dif_th indica el número mínimo de tonos de iluminación entre la imagen
        positiva y la imagen negativa para consisderar que un punto pertenece a la
        escena. Este valor debe ser ajustado según la escena.
    v_mask es la máscara de validez actual para cada pixel. Si no se da se consideran que todos son válidos.
   Returns:
    una máscara 0/255 indicando que un punto tiene una profundidad considerada como
        válida.
*/
    cv::Mat
    clip_XYZ_data(cv::Mat const &XYZ,
                  double x_min,
                  double x_max,
                  double y_min,
                  double y_max,
                  double z_min,
                  double z_max,
                  cv::Mat const &v_mask_)
    {
        cv::Mat v_mask = v_mask_;
        if (v_mask.empty())
            v_mask = cv::Mat::zeros(XYZ.size(), CV_8U);
        for (int y = 0; y < XYZ.rows; ++y)
            for (int x = 0; x < XYZ.cols; ++x)
                if (v_mask.at<uchar>(y, x))
                {
                    const cv::Vec3d p = XYZ.at<cv::Vec3d>(y, x);
                    const bool v = ((p[0] >= x_min && p[0] <= x_max) && (p[1] >= y_min && p[1] <= y_max) && (p[2] >= z_min && p[2] <= z_max));
                    if (!v)
                        v_mask.at<uchar>(y, x) = 0;
                }
        return v_mask;
    }

} //namespace fsiv
