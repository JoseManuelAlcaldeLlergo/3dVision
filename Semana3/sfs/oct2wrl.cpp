#include<iostream>
#include<fstream>
#include<sstream>
#include<cstdlib>
#include<stdexcept>
#include <opencv2/core.hpp>
#include "sfs.hpp"


const cv::String keys =
    "{help h usage ? |      | print this message.}"
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
      parser.about("Save an octree in wrl format.");
      if (parser.has("help"))
      {
          parser.printMessage();
          return 0;
      }

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
      fsiv::Octree octree;
      input >> octree;
      fsiv::save_as_cubes_WRML(output, octree);
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
