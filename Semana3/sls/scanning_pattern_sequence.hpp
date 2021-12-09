#pragma once
#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace fsiv {


struct ScanningPatternSequence
{
    virtual ~ScanningPatternSequence();

    /** @brief Obtiene una copia del objeto. */
    virtual std::shared_ptr<ScanningPatternSequence> clone() const = 0;

    /** @brief Guarda la secuencia en un fichero **/
    virtual bool save(const std::string& fname) const
    {
        //Esta función debe redefinirse en cada sub clase.
        return false;
    };

    /** @brief Descodifica un escaneo.*/
    virtual void decode_scanning(cv::Mat& x_codes, cv::Mat & y_codes) const
    {
        //Esta función debe redefinirse en cada sub clase.
    }

    std::vector<cv::Mat> seq; /*!< secuencia de patrones a proyectar/capturados.*///TOdas las imagenes. Las dos primeras las negativa y positiva, luego ya por bits significativos (de más a menos significativos creo)
};

/** @brief Carga una secuencia de patrones desde un fichero.
    @param fname es el path del fichero.
    @return un puntero al objeto cargado o nullptr si hubo error.
**/
template <class T>
std::shared_ptr<ScanningPatternSequence> load(const std::string& fname);

/** @brief muestra en una ventana todos los patrones de la secuencia **/
void show_scanning(const std::shared_ptr<ScanningPatternSequence>& scan);

}
