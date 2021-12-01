#include<iostream>
#include<fstream>
#include<sstream>
#include<cstdlib>
#include<stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "sfs.hpp"


const cv::String keys =
    "{help h usage ? |      | print this message.}"
    "{cubes          |      | save only externals voxels.}"
    "{@input         |<none>| input voxel set data.}"
    "{@output        |<none>| output .wrl file.}"
    ;

int
main (int argc, char* const* argv)
{
  int retCode=EXIT_SUCCESS;

  try
  {
      cv::CommandLineParser parser(argc, argv, keys);
      parser.about("Save a visual hull as a point cloud in wrl format.");
      if (parser.has("help"))
      {
          parser.printMessage();
          return 0;
      }

      fsiv::VoxelSet vs;
      std::ifstream input (parser.get<std::string>("@input"));
      if (!input)
      {
          std::cerr << "Error: could not open the file ["
                    <<parser.get<std::string>("@input")
                   << "] to read." << std::endl;
          return EXIT_FAILURE;
      }
      std::ofstream output(parser.get<std::string>("@output"));
      if (!output)
      {
          std::cerr << "Error: could not open the file ["
                    <<parser.get<std::string>("@output")
                   << "] to write." << std::endl;
          return EXIT_FAILURE;
      }
      input >> vs;
      if (parser.has("cubes"))
        fsiv::save_as_cubes_WRML(output, vs);
      else
        fsiv::save_as_pointcloud_WRML(output, vs);
  }
  catch (std::exception& e)
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
