#pragma once
#include <memory>
#include <vector>
#include <opencv2/core.hpp>
#include "scanning_pattern_sequence.hpp"

namespace fsiv {

/**
 * @brief Patrón para calibrar el proyector de un SLS.
 * La idea es usar un tablero de calibración impreso en ROJO/AZUL de forma
 * que al iluminar con luz blanca sea vea gris completamente y si se ilumina con
 * color rojo se vea cuadros blancos (los impresos en rojo) y negros (los impresos
 * en azul).
 *
 * Se proyectan dos patrones (por vista):
 *
 * El primero es un tablero B/W usado para detectar los puntos esquinas en la
 * imagen capturada y calcular la homografía de la cámara al proyector.
 *
 * El segundo es una imagen plana en rojo para capturar el tablero de calibración
 * y al aplicarle la homografía calcular, obtener una imagen virtual del tablero
 * de calibración "capturado" por el proyector y de esta forma se puede
 * calibrar el proyector tanto sus parámetros intrínsecos (tomando más de una vista)
 * como los extrínsecos.
 */
struct CalibrationPatternSeq: public ScanningPatternSequence
{
    /** @brief Crea una secunecia vacía. **/
    CalibrationPatternSeq();

    /** @brief Genera una secuencia de dos patrones para calibrar usando un tablero en color rojo/azul.
     * @param img_size image size.
     * @param pattern_size squared pattern size (cols x rows).
     */
    CalibrationPatternSeq(const cv::Size & img_shape,
                          const cv::Size & pattern_shape);

    /** @brief Destructor. **/
    virtual ~CalibrationPatternSeq();

    /** @brief Genera una secuencia de dos patrones para calibrar usando un tablero en color rojo/azul.
     * @param img_size image size.
     * @param pattern_size squared pattern size (cols x rows).
     */
    static std::shared_ptr<CalibrationPatternSeq> create(const cv::Size & img_shape,
                                                         const cv::Size & pattern_shape);
    /** @brief Obtiene una copia del objeto. */
    virtual std::shared_ptr<ScanningPatternSequence> clone() const;

    /** @brief Guarda la secuencia en un fichero **/
    virtual bool save(const std::string& fname) const;

    cv::Size prj_size; /** Geometría del proyector */
    cv::Size board_size; /** geometría de los puntos interiores del tablero proyectado.*/
    std::vector<cv::Point2f> corners; /** coordenadas 2d de las esquinas **/
};

template<>
std::shared_ptr<ScanningPatternSequence> load<CalibrationPatternSeq>(const std::string& fname);

/** @brief Busca un tablero de ajedrez para calcular las coordenadas de los
puntos interiores.
Params:
    img es la imagen con el tablero.
    wnd_name es la ventana para visualizar el resultado.
    cb_size define la geometría de los puntos interios (cols, rows)
Return:
    (was_found, corners) si was_found es True corners son las coordenadas
    subpixel de los puntos interiores.
*/
bool find_chessboard(const cv::Mat& img,
                const cv::Size& cb_size,
                std::vector<cv::Point2f>& corners, const char *wname=nullptr);

/** @brief Dada la geometría de un tablero de calibración, se generan los
correspondientes puntos 3D asumiendo que la esquina superiror derecha del
 tablero es el sistema de coordenadas del mundo (WCS).
*/
std::vector<cv::Point3f>
create_calibration_object_points(const cv::Size& cb_size,
                          float square_size);

} //namespace fsiv
