#pragma once
#include <memory>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include "scanning_pattern_sequence.hpp"

namespace fsiv {


struct BinaryCodeScanning: public ScanningPatternSequence
{
    /** @brief Crea una secuencia vacía. **/
    BinaryCodeScanning();

    /**
     * @brief Genera una secuencia de patrones con codificación binaria.
     *
    Las primeras imagenes son blanca y negra para calcular un umbral por pixel.
    Dependiendo del valor de axis se codifica sólo la y (axis=0), sólo
    la x (axis=1) o ambas (axis=2)
    Para cada eje codificado, se codifican los ceil(log_2(axis_size))-remove_lsb
    bits más significativos.
    Además si use_inverse=True, por cada patrón se genera el inverso.
    Si use_gray_code=true se codifica el código gray en vez del binario puro.
    */
    BinaryCodeScanning(const cv::Size& prj_size,
                       int axis=0,
                       int remove_lsb=2,
                       bool use_inverse=false,
                       bool use_gray_code=false,
                       uchar black_v=0,
                       uchar white_v=255);

    /** @brief destructor. **/
    virtual ~BinaryCodeScanning();

    /**
     * @brief Genera una secuencia de patrones con codificación binaria.
     *
    Las primeras imagenes son blanca y negra para calcular un umbral por pixel.
    Dependiendo del valor de axis se codifica sólo la y (axis=0), sólo
    la x (axis=1) o ambas (axis=2)
    Para cada eje codificado, se codifican los ceil(log_2(axis_size))-remove_lsb
    bits más significativos.
    Además si use_inverse=True, por cada patrón se genera el inverso.
    Si use_gray_code=True se codifica el código gray en vez del binario puro.
    */
    static std::shared_ptr<BinaryCodeScanning> create(const cv::Size& prj_size,
                                                      int axis=0,
                                                      int remove_lsb=2,
                                                      bool use_inverse=false,
                                                      bool use_gray_code=false,
                                                      uchar black_v=0,
                                                      uchar white_v=255);
    /** @brief Obtiene una copia del objeto. */
    virtual std::shared_ptr<ScanningPatternSequence> clone() const;

    /** @brief Guarda la secuencia en un fichero **/
    virtual bool save(const std::string& fname) const;

    virtual void decode_scanning(cv::Mat & x_codes, cv::Mat& y_codes) const;

    int axis; /*!< which axis: 0:vertical, 1:horizontal, 2->both. */
    cv::Size prj_size; /*!< projector image size WxH.*/
    int remove_lsb; /*!< number of lsb bits which are not codified.*/
    bool use_inverse; /*!< Is there a inverse image for each pattern?.*/
    bool use_gray_code; /*!< the patterns codify binary gray code. NI CASO*/
};

/** @brief Carga un escaneo desde fichero. **/
template<>
std::shared_ptr<ScanningPatternSequence> load<BinaryCodeScanning>(const std::string& fname);


}
