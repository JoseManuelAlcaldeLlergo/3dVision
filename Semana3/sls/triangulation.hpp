#pragma once

#include <opencv2/core.hpp>
#include "cparams.hpp"

namespace fsiv {

/**
 * @brief Calcula la triangulación con el esquema intersección recta-plano.
 * @param p_codes son los codigos de plano vertical/horizontal decodificados.
 * @param axis indica si los planos son verticales axis=0 codificando coord. x u horizontales axis=1 (coord. y).
 * @param cparams son los parámetros de calibración del sistema.
 * @param mask es una imagen 0|255 para indicar sobre que puntos calcular las coordenadas.
 * @return return depth_m es el mapa de profundidad calculado.
 */
cv::Mat compute_line_plane_triangulation(cv::Mat const&p_codes, int  axis,
                                         CParams const& cparams,
                                         const cv::Mat & mask=cv::Mat());

/**
 * @brief Calcula la triangulación con el esquema intersección recta-recta.
 * @param x_codes son los códigos decodificados de la coordenada x del proyector
 * @param y_codes son los códigos decodificados de la coordenada y del proyector
 * @param cparams son los parámetros de calibración del sistema.
 * @param mask es una imagen 0|255 para indicar sobre que puntos calcular las coordenadas.
 * @return return mapa de profundidad XYZ calculado.
 */
cv::Mat compute_line_line_triangulation(cv::Mat const&x_codes,
                                        cv::Mat const&y_codes,
                                        CParams const& cparams,
                                        const cv::Mat &mask = cv::Mat());

/** @brief Calcula las coordenadas XYZ en WCS de los puntos proyectados en el
 *     plano imagen de la cámara conocidas sus profundidades medias en WCS.
 * @param depth_m es el mapa de profundidad.
 * @param cparams son los parámetros de calibración.
 * @param mask es una imagen 0|255 para indicar sobre que puntos calcular las coordenadas.
 * @return a matrix with 3 channels X,Y,Z of each point.
 */
cv::Mat depth_map_to_XYZ(cv::Mat const& depth_m, CParams const& cparams,
                         const cv::Mat &mask=cv::Mat());

/** @brief Guarda una nube de puntos escaneada en formato VRML2.0
 *
    Params:
        f es un stream donde escribir.
        XYZ es el mapa de puntos 3D.
        img_color es la imagen con el color de los puntos del mapa.
        validity_mask indica si un punto del mapa es válido o no.
*/
void save_XYZ_to_vrml(std::ostream& f,
                      cv::Mat const& XYZ,
                      cv::Mat const& img_color,
                      cv::Mat const& validity_mask_=cv::Mat());

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
cv::Mat clip_XYZ_data(cv::Mat const & XYZ,
                      double x_min,
                      double x_max,
                      double y_min,
                      double y_max,
                      double z_min,
                      double z_max,
                      cv::Mat const& v_mask=cv::Mat());

} // namespace fsvi
