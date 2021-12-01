#pragma once
#include <iostream>
#include <opencv2/core.hpp>

namespace fsiv
{

/**
 * @brief The Voxel class.
 * Models a volumen of 3D space.
 */
class Voxel
{
public:

  Voxel();  
  Voxel( const cv::Mat& origin, const cv::Mat& dims);
  Voxel (const float x, const float y, const float z,
     const float wX, const float wY, const float wZ);  

  //x y z representan el punto origen del voxel, y los dim lo que miden en esas direcciones (CREO)
  float x() const;
  float y() const;
  float z() const;
  float x_dim() const;
  float y_dim() const;
  float z_dim() const;
  cv::Mat origin() const;
  cv::Mat dims() const;

  /**
   * @brief get the center point of the voxel.
   *  @return the point as 3x1 matrix of floats.
   */
  cv::Mat center() const;

  /**
   * @brief get the XYZ coordinates of the eight vertices of a voxel.
   * @return a 3x8 float matrix with the vertices.
   */
  cv::Mat vertices() const;

  double volume() const;
  void set(const cv::Mat& origin, const cv::Mat& dims);
  void set(const float x, const float y, const float z,
     const float wX, const float wY, const float wZ);

private:
  float x_, y_, z_; //Punto origen(creo)
  float wx_, wy_, wz_; //Anchos(creo)
};

/** @brief Save a voxel to a stream.*/
std::ostream& operator<<(std::ostream& out, const Voxel& voxel);
/** @brief Load a voxel from a stream.*/
std::istream& operator>>(std::istream& in, Voxel& voxel);

} //namespacde fsiv
