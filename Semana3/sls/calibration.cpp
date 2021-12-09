#include "calibration.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

namespace fsiv {

/** @brief Dada la geometría de un tablero de calibración, se generan los
correspondientes puntos 3D asumiendo que la esquina superiror derecha del
 tablero es el sistema de coordenadas del mundo (WCS).
*/
std::vector<cv::Point3f>
create_calibration_object_points(const cv::Size& cb_size,
                          float square_size)
{
    std::vector<cv::Point3f> points;
    for (int y=1; y<=cb_size.height; ++y)
        for (int x=1; x<=cb_size.width; ++x)
            points.push_back(cv::Point3f(x*square_size, y*square_size, 0.0));
    return points;
}

/**
 * @brief Genera una imagen flotante con un chessboard para mostrar con el projector.
    Params:
        prj_size es la geometría del proyector.
        cb_size es el número de puntos interiores del tablero (cols, fils)
        border_width es el ancho de borde dejado alrededor del tablero.
        white_color es el color usado para los cuadros "blancos".
        black_color es el color para los cuadros "negros"
    Returns:
        img, la image a proyectar.
        points, las coordenadas (x,y) de los puntos interiores en recorrido.
*/
static void
gen_squares_image(const cv::Size& prj_size,
                  const cv::Size& cb_size,
                  cv::Mat& img,
                  std::vector<cv::Point2f>& points,
                  const int border_width=25,
                  const cv::Scalar& white_color = cv::Scalar::all(255),
                  const cv::Scalar& black_color = cv::Scalar::all(0))
{
    int squared_height = (prj_size.height-(2*border_width)) / (cb_size.height+1);
    int squared_width = (prj_size.width-(2*border_width)) / (cb_size.width+1);
    img = cv::Mat(prj_size, CV_8UC3, white_color);
    points.resize(0);
    for (int r=0; r<=cb_size.height; ++r)
        for (int c=0; c<=cb_size.width; ++c)
        {
            if ((r+c)%2 == 0)
            {
                img(cv::Range(border_width+r*squared_height,
                              border_width+(r+1)*squared_height),
                    cv::Range(border_width+c*squared_width,
                              border_width+(c+1)*squared_width)) = black_color;
            }
            if (r>=1 and c>=1)
                points.push_back(cv::Point2f(border_width+c*squared_width,
                                             border_width+r*squared_height));
        }
    return;
}

/** @brief Busca un tablero de ajedrez para calcular las coordenadas de los
puntos interiores.
Params:
    img es la imagen con el tablero.
    wnd_name es la ventana para visualizar el resultado.
    cb_size define la geometría de los puntos internos (cols, rows)
Return:
    (was_found, corners) si was_found es True corners son las coordenadas
    subpixel de los puntos interiores.
*/
bool
find_chessboard(const cv::Mat& img,
                const cv::Size& cb_size,
                std::vector<cv::Point2f>& corners,
                const char * wname)
{
    bool was_found = cv::findChessboardCorners(img, cb_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH );
    if (was_found)
    {
        cv::Mat img_;
        if (img.channels()==3)
            cv::cvtColor(img, img_, cv::COLOR_BGR2GRAY);
        else
            img_ = img;
        cv::cornerSubPix(img_, corners, cv::Size(5, 5),
                         cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS +
                                          cv::TermCriteria::MAX_ITER,
                                          30, 0.001));
    }
    if(wname!=nullptr)
    {
        cv::Mat img_aux;
        if (img.channels()==1)
            cv::cvtColor(img, img_aux, cv::COLOR_GRAY2BGR);
        else
            img_aux=img.clone();
        cv::drawChessboardCorners(img_aux, cb_size, corners, was_found);
        cv::namedWindow(wname, cv::WINDOW_AUTOSIZE);
        cv::imshow(wname, img_aux);
        cv::waitKey(0);
        cv::destroyWindow(wname);
    }
    return was_found;
}

CalibrationPatternSeq::CalibrationPatternSeq()
{}

CalibrationPatternSeq::CalibrationPatternSeq(const cv::Size & prj_size_,
                                         const cv::Size & board_size_)
{
    cv::Mat img;
    prj_size = prj_size_;
    board_size = board_size_;
    corners.resize(0);
    gen_squares_image(prj_size, board_size, img, corners);
    seq.push_back(img);
    seq.push_back(cv::Mat(prj_size, CV_8UC3, cv::Scalar(0.0, 0.0, 255)));
    return;
}

std::shared_ptr<CalibrationPatternSeq>
CalibrationPatternSeq::create(const cv::Size & prj_size,
                              const cv::Size & board_size)
{
    return std::make_shared<CalibrationPatternSeq>(prj_size, board_size);
}

std::shared_ptr<ScanningPatternSequence>
CalibrationPatternSeq::clone() const
{
    auto pat = std::make_shared<CalibrationPatternSeq>();
    pat->prj_size = prj_size;
    pat->board_size = board_size;
    pat->corners = corners;
    for (size_t i=0;i<seq.size();++i)
        pat->seq.push_back(seq[i].clone());
    std::shared_ptr<ScanningPatternSequence> ret_v = pat;
    return ret_v;
}

/** @brief Guarda la secuencia en un fichero **/
bool CalibrationPatternSeq::save(const std::string& fname) const
{
    bool was_ok = true;
    auto file = cv::FileStorage();
    was_ok = file.open(fname, cv::FileStorage::WRITE);
    if (was_ok)
    {
        file << "prj_size" << prj_size;
        file << "board_size"  << board_size;
        file << "corners" << corners;
        int n_images = seq.size();
        file << "nun-images" << n_images;
        std::ostringstream image_label;
        for (int i=0;i<n_images;++i)
        {
            image_label.str("");
            image_label << "image-" << i;
            file << image_label.str() << seq[i];
        }
    }
    return was_ok;
}

template<>
std::shared_ptr<ScanningPatternSequence>
load<CalibrationPatternSeq>(const std::string& fname)
{
    std::shared_ptr<CalibrationPatternSeq> bc_scan;
    bool was_ok = true;
    auto file = cv::FileStorage();
    was_ok = file.open(fname, cv::FileStorage::READ);
    if (was_ok)
    {
        bc_scan = std::make_shared<CalibrationPatternSeq>();
        file["prj_size"] >> bc_scan->prj_size;
        file["board_size"] >> bc_scan->board_size;
        file["corners"] >> bc_scan->corners;
        int n_images;
        file["nun-images"] >> n_images;
        std::ostringstream image_label;
        for (int i=0;i<n_images;++i)
        {
            image_label.str("");
            image_label << "image-" << i;
            cv::Mat img;
            file[image_label.str()] >> img;
            bc_scan->seq.push_back(img);
        }
    }
    std::shared_ptr<ScanningPatternSequence> ret_v = bc_scan;
    return ret_v;
}

CalibrationPatternSeq::~CalibrationPatternSeq()
{}

} //namespace fsiv
