#include "bc_scanning.hpp"
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "sls.hpp"
#include <iostream>

namespace fsiv
{

    template <>
    std::shared_ptr<ScanningPatternSequence>
    load<BinaryCodeScanning>(const std::string &fname)
    {
        std::shared_ptr<BinaryCodeScanning> bc_scan;
        bool was_ok = true;
        auto file = cv::FileStorage();
        was_ok = file.open(fname, cv::FileStorage::READ);
        if (was_ok)
        {
            bc_scan = std::make_shared<BinaryCodeScanning>();
            file["axis"] >> bc_scan->axis;
            int prj_widht, prj_height;
            file["prj-width"] >> prj_widht;
            file["prj-height"] >> prj_height;
            bc_scan->prj_size = cv::Size(prj_widht, prj_height);
            file["use-inverse"] >> bc_scan->use_inverse;
            file["use-gray-code"] >> bc_scan->use_gray_code;
            file["remove-lsb"] >> bc_scan->remove_lsb;
            int n_images;
            file["nun-images"] >> n_images;
            std::ostringstream image_label;
            for (int i = 0; i < n_images; ++i)
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

    bool
    BinaryCodeScanning::save(const std::string &fname) const
    {
        bool was_ok = true;
        auto file = cv::FileStorage();
        was_ok = file.open(fname, cv::FileStorage::WRITE);
        if (was_ok)
        {
            file << "axis" << axis;
            file << "prj-width" << prj_size.width;
            file << "prj-height" << prj_size.height;
            file << "use-inverse" << use_inverse;
            file << "use-gray-code" << use_gray_code;
            file << "remove-lsb" << remove_lsb;
            int n_images = seq.size();
            file << "nun-images" << n_images;
            //Only the two first images must be saved as rgb.
            file << "image-0" << seq[0];
            file << "image-1" << seq[1];
            std::ostringstream image_label;
            cv::Mat img_g;
            for (int i = 2; i < n_images; ++i)
            {

                image_label.str("");
                image_label << "image-" << i;
                if (seq[i].channels() == 3)
                    cv::cvtColor(seq[i], img_g, cv::COLOR_BGR2GRAY);
                else
                    img_g = seq[i];
                file << image_label.str() << img_g;
            }
        }
        return was_ok;
    }

    /**
 * @brief Decodifica un plano de bit en un patron usado con codificacion binaria.
    Params:
        img_pos es la captura correspondiente a la proyección del patron positivo.
        img_neg es la captura correspondiete a la proyección del patrón negativo
            o a la imagen con valores medios.
        bit which bit position is being decoded.
    Returns:
        matriz de tipo CV_16SC1 con el valor correspondiente al bit decodificado
            de cada punto.
*/
    cv::Mat
    decode_binary_code_pattern(const cv::Mat &img_pos, const cv::Mat &img_neg, int bit)
    {
        CV_Assert(img_pos.type() == CV_8UC1 && img_pos.type() == img_neg.type());
        CV_Assert(img_pos.size() == img_neg.size());
        CV_Assert(bit >= 0);
        cv::Mat decoding;
        //TODO
        //Recuerda en la imagen de salida los pixeles "iluminados" por el proyector
        //deberán tener el valor 2^bit en la imagen devuelta.
        //Utiliza opencv para generar código vectorizado (sin bucles).

        // Creamos una matriz de 0s para rellenarla rapidamente con la suma vectorial
        cv::Mat out = cv::Mat::zeros(img_pos.rows, img_pos.cols, CV_16SC1);

        cv::Mat mask = img_pos >= img_neg;

        // Ahora en out tenemos una matriz completamente rellena de 2^bit
        out += pow(2, bit);

        // Ponemos en decoding 2^bit solamente en los pixeles iluminados
        out.copyTo(decoding, mask);

        //
        CV_Assert(decoding.size() == img_pos.size() && decoding.type() == CV_16SC1);
        return decoding;
    }

    /** @brief Convierte de código gray a binario.
    @warning Ojo sólo para enteros de 32bits.
 */
    static cv::Mat
    convert_gray_to_binary_code(const cv::Mat &img)
    {
        CV_Assert(img.type() == CV_16SC1);
        cv::Mat ret = img.clone();
        for (auto v = ret.begin<cv::int16_t>(); v != ret.begin<cv::int16_t>(); ++v)
        {
            auto num = *v;
            num ^= num >> 16;
            num ^= num >> 8;
            num ^= num >> 4;
            num ^= num >> 2;
            num ^= num >> 1;
            *v = num;
        }
        return ret;
    }

    void
    BinaryCodeScanning::decode_scanning(cv::Mat &x_codes, cv::Mat &y_codes) const
    {
        cv::Mat mean_img, pos_img, neg_img;
        //TODO
        //Revisa los atributos de la clase que son necesarios para decodificar.
        pos_img = seq[0].clone();
        neg_img = seq[1].clone();


        //TODO
        //Primero, si la propiedad use_inverse no es cierta, calcula la imagen
        //media entre la imagen de la escenea positivo (sep[0]) y la negativa (sep[1])
        //
        if (!use_inverse)
        {
            //Calculo de la imagen media mediante operaciones vectoriales
            mean_img = (pos_img + neg_img) / 2;
        }
        //
        CV_Assert(use_inverse || (mean_img.size() == seq[0].size() && mean_img.type() == CV_8UC1));

        //Los escaneos comienzan desde el índice 2.

        size_t seq_idx = 2;
        //

        if (axis == 0 || axis == 2)
        {

            //Si el atributo axis==0 o axis==2, los patrones a partir de seq_idx
            //son la codificación usando patrones horizontales que codifican
            //la coordenada y de fila.
            //
            //Recuerda que se codifica desde el bit más siginificativo hasta
            //el el valor del atributo remove_lsb inclusive.
            //
            //Recuerda que si la propiedad use_inverse es cierta, cada bit usará
            //dos patrones de la secuencia: sep[seq_idx]=positivo, seq[seq_idx+1]=negativo.
            //En caso contrario sólo tendremos uno positivo y usaremos la imagen
            //media (mean_img) como patrón negativo para todos los bits.
            //
            //Recuerda en cada iteración, actualizar el índice seq_idx
            //de forma acorde al valor use_inverse.

            y_codes = cv::Mat::zeros(seq[0].rows, seq[0].cols, CV_16SC1);

            //TODO
            //Para cada posición de bit desde la más significativa hasta la
            //posición remove_lsb inclusive, hacer ...
            //Sugerencia: utiliza la función fsiv::decode_binary_code_pattern()

            // Recorremos desde el bit más significativo
            int i = int(std::floor(std::log2(prj_size.height)));
            for (; i >= remove_lsb; i--)
            {
                if (use_inverse)
                {
                    y_codes += decode_binary_code_pattern(seq[seq_idx], seq[seq_idx + 1], i);
                    seq_idx += 2;
                }
                else
                {
                    y_codes += decode_binary_code_pattern(seq[seq_idx], mean_img, i);
                    seq_idx += 1;
                }
            }

            //
            if (use_gray_code)
                y_codes = convert_gray_to_binary_code(y_codes);
        }

        if (axis == 1 || axis == 2)
        {
            //Si el atributo axis==1 o axis==2, los patrones a partir de seq_idx
            //son la codificación usando patrones verticales que codifican
            //la coordenada x de columna.
            //
            //Recuerda que se codifica desde el bit más siginificativo hasta
            //el valor del atributo remove_lsb inclusive.
            //
            //Recuerda que si la propiedad use_inverse es cierta, cada bit usará
            //dos patrones de la secuencia: sep[seq_idx]=positivo, seq[seq_idx+1]=negativo.
            //En caso contrario sólo tendremos uno positivo y usaremos la imagen
            //media (mean_img) como patrón negativo para todos los bits.
            //
            //Recuerda en cada iteración, actualizar el índice seq_idx
            //de forma acorde al valor use_inverse.

            x_codes = cv::Mat::zeros(seq[0].rows, seq[0].cols, CV_16SC1);

            //TODO:
            //Para cada posición de bit desde la más significativa hasta la
            //posición remove_lsb inclusive, hacer ...
            //Sugerencia: utiliza la función fsiv::decode_binary_code_pattern()

            // Ahora igual, pero cogiendo width porque es para las líneas verticales
            int i = int(std::floor(std::log2(prj_size.width)));
            for (; i >= remove_lsb; i--)
            {
                if (use_inverse)
                {
                    x_codes += decode_binary_code_pattern(seq[seq_idx], seq[seq_idx + 1], i);
                    seq_idx += 2;
                }
                else
                {
                    x_codes += decode_binary_code_pattern(seq[seq_idx], mean_img, i);
                    seq_idx += 1;
                }
            }

            //
            if (use_gray_code)
                x_codes = convert_gray_to_binary_code(x_codes);
        }
        //
        CV_Assert(axis == 1 || (x_codes.type() == CV_16SC1 && x_codes.size() == seq[0].size()));
        CV_Assert(axis == 0 || (y_codes.type() == CV_16SC1 && y_codes.size() == seq[0].size()));
    }

    /**
 * @brief Convierte de codificación binaria a codigo gray.
 * @param binary_code
 * @return transformed code.
 */
    static int
    convert_binary_to_gray_code(int binary_code)
    {
        return binary_code ^ (binary_code >> 1);
    }

    /** @brief Genera un patron de codificación binaria.
Params:
    image_shape dimensión de la imagen a generar.
    bit_position es la posición de bit que codifica el patrón siendo la
          posición 0 la del bit menos significativo.
    axis indica el eje a codificar. axis=0 las filas, axis=1 las columnas.
    gray_code indica si usar codificación gray (True) o raw (False)
    black_v, white_v indican los colores a usar para "blanco" y "negro".
Returns:
    El patrón.
**/
    static cv::Mat
    create_binary_code_pattern(const cv::Size &image_size,
                               int bit_position,
                               int axis = 0,
                               bool use_gray_code = false,
                               uchar black_v = 0,
                               uchar white_v = 255)
    {
        cv::Mat pattern = cv::Mat(image_size, CV_8UC1, black_v);
        int plane_bit = 1 << bit_position;
        if (axis == 1) //Vertical planes.
        {
            cv::Mat line = cv::Mat(1, image_size.width, CV_8UC1, black_v);
            for (int x = 0; x < image_size.width; ++x)
            {
                int code = x;
                if (use_gray_code)
                    code = convert_binary_to_gray_code(code);
                if (code & plane_bit)
                    line.at<uchar>(x) = white_v;
            }
            for (int y = 0; y < image_size.height; ++y)
                line.copyTo(pattern(cv::Range(y, y + 1), cv::Range::all()));
        }
        else if (axis == 0) //horizontal planes.
        {
            cv::Mat line = cv::Mat(1, image_size.width, CV_8UC1, white_v);
            for (int y = 0; y < image_size.height; ++y)
            {
                int code = y;
                if (use_gray_code)
                    code = convert_binary_to_gray_code(code);
                if (code & plane_bit)
                    line.copyTo(pattern(cv::Range(y, y + 1), cv::Range::all()));
            }
        }
        return pattern;
    }

    BinaryCodeScanning::BinaryCodeScanning()
    {
    }

    BinaryCodeScanning::BinaryCodeScanning(const cv::Size &prj_size_,
                                           int axis_,
                                           int remove_lsb_,
                                           bool use_inverse_,
                                           bool use_gray_code_,
                                           uchar black_v,
                                           uchar white_v)
    {
        axis = axis_;
        prj_size = prj_size_;
        remove_lsb = remove_lsb_;
        use_gray_code = use_gray_code_;
        use_inverse = use_inverse_;
        seq.push_back(cv::Mat(prj_size, CV_8UC1, white_v));
        seq.push_back(cv::Mat(prj_size, CV_8UC1, black_v));
        if (axis == 0 || axis == 2)
        {
            int plane_bit = int(std::floor(std::log2(prj_size.width)));
            while (plane_bit >= remove_lsb)
            {
                seq.push_back(create_binary_code_pattern(prj_size, plane_bit, 0,
                                                         use_gray_code, black_v,
                                                         white_v));
                if (use_inverse)
                    seq.push_back(create_binary_code_pattern(prj_size, plane_bit,
                                                             0, use_gray_code,
                                                             white_v, black_v));
                --plane_bit;
            }
        }
        if (axis == 1 || axis == 2)
        {
            int plane_bit = int(std::floor(std::log2(prj_size.height)));
            while (plane_bit >= remove_lsb)
            {
                seq.push_back(create_binary_code_pattern(prj_size, plane_bit, 1,
                                                         use_gray_code, black_v,
                                                         white_v));
                if (use_inverse)
                    seq.push_back(create_binary_code_pattern(prj_size, plane_bit,
                                                             1, use_gray_code,
                                                             white_v, black_v));
                --plane_bit;
            }
        }
    }

    std::shared_ptr<BinaryCodeScanning>
    BinaryCodeScanning::create(const cv::Size &prj_size, int axis, int remove_lsb,
                               bool use_inverse, bool use_gray_code,
                               uchar black_v, uchar white_v)
    {
        return std::make_shared<BinaryCodeScanning>(prj_size, axis, remove_lsb,
                                                    use_inverse, use_gray_code,
                                                    black_v, white_v);
    }

    std::shared_ptr<ScanningPatternSequence>
    BinaryCodeScanning::clone() const
    {
        auto bc_patt = std::make_shared<BinaryCodeScanning>();
        bc_patt->axis = axis;
        bc_patt->prj_size = prj_size;
        bc_patt->remove_lsb = remove_lsb;
        bc_patt->use_gray_code = use_gray_code;
        bc_patt->use_inverse = use_inverse;
        for (size_t i = 0; i < seq.size(); ++i)
            bc_patt->seq.push_back(seq[i].clone());
        std::shared_ptr<ScanningPatternSequence> ret_v = bc_patt;
        return ret_v;
    }

    BinaryCodeScanning::~BinaryCodeScanning()
    {
    }

} //namespace fsiv
