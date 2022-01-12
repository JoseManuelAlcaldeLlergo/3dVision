// "args": ["--scene=-1.5:-1.5:0.0:3:3:2", "--vsize=0.02","--output=my_voxelset", "--nviews=4", "./data/cone-4/ext_4-0.yml", "./data/cone-4/ext_4-1.yml", "./data/cone-4/ext_4-2.yml", "./data/cone-4/ext_4-3.yml", "./data/cone-4/cone_4-0.png", "./data/cone-4/cone_4-1.png", "./data/cone-4/cone_4-2.png", "./data/cone-4/cone_4-0.png" ]

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "sfs.hpp"

const cv::String keys =
    "{help h usage ? |      | print this message.}"
    "{verbose        |0     | Verbose level.}"
    "{oar_th         |0.5   | Occupancy area rate.}"
    "{scene          |<none>| Set the scene dimensions in WCS units. Format xorig:yorig:zorig:xsize:ysize:zsize}"
    "{vsize          |<none>| Set the voxel side size in WCS units.}"
    "{output         |<none>| Output file to save the computed voxel set.}"
    "{nviews         |<none>| Number of views.}"
    "{@cam_0         |<none>| Camera parameters for view 0...}"
    "{@cam_n         |<none>| ... camera parameters for view N.}"
    "{@view_0        |<none>| Foreground image for view 0...}"
    "{@view_n        |<none>| ... foreground image for view N.}";

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;

    try
    {
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Build a visual hull by computing the voxelset.");
        if (parser.has("help"))
        {
            parser.printMessage();
            return 0;
        }

        fsiv::DebugLevel_ = parser.get<int>("verbose");
        std::istringstream buffer(parser.get<std::string>("scene"));
        fsiv::Voxel scene;
        buffer >> scene;
        if (!buffer)
        {
            std::cerr << "Error: Worng: cli parameter scene." << std::endl;
            return EXIT_FAILURE;
        }
        float voxel_size = parser.get<float>("vsize");
        size_t n_views = parser.get<int>("nviews");
        std::ofstream output(parser.get<std::string>("output"));
        std::ofstream output_wrl(parser.get<std::string>("output")+".wrl");
        if (!output)
        {
            std::cerr << "Error: could not open file ["
                      << parser.get<std::string>("@output")
                      << "] to write." << std::endl;
            return EXIT_FAILURE;
        }
        size_t first_arg = 1;
        while (first_arg < argc && argv[first_arg][0] == '-')
            ++first_arg;
        if ((argc - first_arg) != n_views * 2)
        {
            std::cerr << "Error: wrong cli." << std::endl;
            return EXIT_FAILURE;
        }
        std::vector<fsiv::View> views;
        std::ostringstream view_name;
        for (size_t v = 0; v < n_views; ++v)
        {
            view_name.str("");
            view_name << "View " << v;
            fsiv::CameraParameters cparams;
            if (!cparams.read_from_file(argv[first_arg + v]))
            {
                std::cerr << "Error: could not load camera parameters form file["
                          << argv[5 + v] << "]." << std::endl;
                return EXIT_FAILURE;
            }
            cv::Mat fg_img = cv::imread(argv[first_arg + n_views + v], cv::IMREAD_GRAYSCALE);
            // std::cout<<"nombre vista: "<<argv[first_arg + n_views + v]<<std::endl;
            // cv::imshow("VISTA",fg_img);
            // cv::waitKey(0);
            if (fg_img.empty())
            {
                std::cerr << "Error: could not load forground image form file["
                          << argv[5 + n_views + v] << "]." << std::endl;
                return EXIT_FAILURE;
            }
            views.push_back(fsiv::View(view_name.str(), fg_img, cparams));
        }
        fsiv::VoxelSet vs;
        fsiv::compute_visual_hull(views, vs, scene, voxel_size,
                                  parser.get<float>("oar_th"));
        output << vs;

        fsiv::save_as_pointcloud_WRML(output_wrl, vs);
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    catch (...)
    {
        std::cerr << "Capturada excepcion desconocida!" << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
