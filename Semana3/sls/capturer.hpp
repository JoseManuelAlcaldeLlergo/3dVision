#pragma once
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include "projector.hpp"
#include "scanning_pattern_sequence.hpp"

namespace fsiv {

class Capturer
{
public:
    Capturer(int c_idx);
    Capturer(int c_idx, cv::FileStorage& cparams);
    bool open(int c_idx, cv::FileStorage& cparams);
    ~Capturer();
    bool is_opened() const;
    /** @brief Muestra video en vivo en una ventana hasta que el usuario pulse alguna tecla.
    Params:
        cam es un objeto cv2.VideoCapture.
        wnd_name es el nombre de la ventana.
    Returns:
        (wasOk, key) si se puede ver video wasOK es true y key alamacena la tecla
        pulsada por el usuario para terminar.
    */
    bool show_live_video(const char *wname, int* key);

    /** @brief Realiza una secuencia de proyección/scan de una secuenciad e patrones patrones.
    Params:
        prj es el projector a usar.
        patterns[in,out] es la secuencia de patrones a projectar y escanear.
    Returns:
        wasOk, scanning) wasOk es True si se puedo realizar la operación y
        scanning es una lista con las imágenes escaneadas.
    */
    bool scan_pattern_sequence(Projector& prj,
                               std::shared_ptr<ScanningPatternSequence>& pattern_seq);
    /** @brief Captura una imagen de una cámara.
    Params:
        img es la imagen donde guardar la captura.mediar para generar una.
    Returns:
        wasOk indicando si se realizó la captura.
    */
    bool capture_image(cv::Mat& img_);
private:

    cv::VideoCapture cap_;
    cv::Size img_size_;

    int show_wait_;
    int gaussian_r_;
    int NUM_GRAB_PER_SHOT_;
    int NUM_AVG_IMAGES_;

};
} // namespace fsiv
