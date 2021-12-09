#include <iostream>
#include <fstream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

#include "sls.hpp"

const cv::String keys =
    "{help h usage ? |      | print this message   }"
    "{v verbose      |0     | Verbose level. Value 0 means not log.}"        
    "{cp             |cap_param.yml| Capture parameters file.}"
    "{x_orig         |0     | Projector Window's X origin.}"
    "{y_orig         |0     | Projector Window's Y origin.}"
    "{c              |-1    | Camera device idx.}"
    "{@pattern       |<none>| Pattern to project.}"
    "{@output        |<none>| Output scanning file.}"
    ;

int
main (int argc, char* const* argv)
{
    int retCode=EXIT_SUCCESS;

    try {

        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Projects and capture a pattern scan sequence.");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }

        int verbose = parser.get<int>("v");
        int x_orig = parser.get<int>("x_orig");
        int y_orig = parser.get<int>("y_orig");
        int c_idx = parser.get<int>("c");
        std::string output_fname = parser.get<std::string>("@output");

        cv::FileStorage cparams;
        if (!cparams.open(parser.get<std::string>("cp"), cv::FileStorage::READ))
        {
            std::cerr << "Error: could not open ["
                      << parser.get<std::string>("cp")
                      << "] to read." << std::endl;
            return EXIT_FAILURE;
        }

        fsiv::Capturer capt(c_idx, cparams);
        if (!capt.is_opened())
        {
            std::cerr << "Error: could not open device idx "
                      << c_idx << "." << std::endl;
            return EXIT_FAILURE;
        }

        auto patterns = fsiv::load<fsiv::BinaryCodeScanning>(
                    parser.get<std::string>("@pattern"));
        if (patterns == nullptr)
        {
            std::cerr << "Error: could not open ["
                      << parser.get<std::string>("@pattern")
                      << "] to read." << std::endl;
            return EXIT_FAILURE;
        }

        if (verbose>0)
            fsiv::show_scanning(patterns);

        fsiv::Projector prj(x_orig, y_orig);
        prj.switch_on();


        bool go_out = false;
        bool was_ok = true;
        int scan_idx = 0;
        int key = -1;
        while (!go_out)
        {
            //Mostramos video en vivo para que configurar una nueva pose
            //del sistema.
            prj.switch_on();
            const char * wnd_title="Ajusta. Pulsa tecla (Esc aborta)";
            prj.project(patterns->seq[0], 20);
            was_ok = capt.show_live_video(wnd_title, &key);
            go_out = !was_ok || (key&0xff)==27;
            auto scanning = patterns->clone();
            if (capt.scan_pattern_sequence(prj, scanning))
            {
                prj.switch_off();
                if (verbose>0)
                    fsiv::show_scanning(scanning);
                if (scanning->save(output_fname))
                {
                    std::cout << "Scanning " << scan_idx << " save to: " << output_fname << std::endl;
                }
                scan_idx++;
            }
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}

