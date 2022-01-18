/*!
  Esto es un esqueleto de programa para usar en las prácticas
  de Visión Artificial.

  Se supone que se utilizará OpenCV.

  Para compilar, puedes ejecutar:
    g++ -Wall -o esqueleto esqueleto.cc `pkg-config opencv --cflags --libs`

*/

#include <iostream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

// const cv::String keys =
//     "{help h usage ? |      | print this message   }"
//     "{path           |.     | path to file         }"
//     "{fps            | -1.0 | fps for output video }"
//     "{N count        |100   | count of objects     }"
//     "{ts timestamp   |      | use time stamp       }"
//     "{@image1        |      | image1 for compare   }"
//     "{@image2        |<none>| image2 for compare   }"
//     "{@repeat        |1     | number               }"
//     ;

const cv::String keys =
    "{help h usage ? |      | print this message   }"
    "{@image       |      | image   }"
    ;


/*!
    @brief Imprime los valores mínimo y máximo de una imagen por canales

    @param[in] img es la imagen de entrada.

    @pre img no está vacia.
*/
void print_min_max(cv::Mat img){
  //Comprobacion de precondiciones.
  CV_Assert( !img.empty() );

  //Obtenemos los diferentes canales de la imagen
  cv::Mat channels[3];
  cv::split(img,channels);
  
  double min, max;

  std::cout<<"La imagen tiene "<<img.channels()<<" canal(es)."<<std::endl;

  for(int i=0; i<img.channels(); i++){
    std::string color = "";
    switch (i)
    {
    case 0:
      color = "Blue";
      break;
    case 1:
      color = "Green";
      break;
    case 2:
      color = "Red";
      break;
    
    default:
      color = "Unknow channel";
      break;
    }
    cv::minMaxLoc(channels[i],&min,&max,0,0);
    std::cout<<"Canal "<<i<<"("<<color<<"):\tMin: "<<min<< "  Max: "<<max<<std::endl;
  }
  
}


int
main (int argc, char* const* argv)
{
  int retCode=EXIT_SUCCESS;
  
  try {    

      cv::CommandLineParser parser(argc, argv, keys);
      parser.about("Application name v1.0.0");
      if (parser.has("help"))
      {
          parser.printMessage();
          return 0;
      }
      
      std::cout<<"El programa asume solamente imágenes en escala de grises o en BGR"<<std::endl<<std::endl;

      cv::String img_name = parser.get<cv::String>("@image");
      // cv::String img2 = parser.get<cv::String>("@image2");
      // int repeat = parser.get<int>("@repeat");
      if (!parser.check())
      {
          parser.printErrors();
          return 0;
      }

    /*Ahora toca que tu rellenes con lo que hay que hacer ...*/
      cv::Mat img = cv::imread(img_name, cv::IMREAD_ANYCOLOR);

      print_min_max(img);

      std::cout<<"\nFIN DEL PROGRAMA"<<std::endl;
      

    
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
