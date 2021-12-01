#include "voxel.hpp"

namespace fsiv {

Voxel::Voxel()
{
  x_=0.f;
  y_=0.f;
  z_=0.f;
  wx_=0.f;
  wy_=0.f;
  wz_=0.f;
}

Voxel::Voxel(const cv::Mat& origin, const cv::Mat& dims)
{
    CV_Assert(origin.type()==CV_32FC1);
    CV_Assert((origin.rows*origin.cols)==3);
    CV_Assert((dims.rows*dims.cols)==3);
    set(origin.at<float>(0), origin.at<float>(1), origin.at<float>(2),
        dims.at<float>(0), dims.at<float>(1), dims.at<float>(2));
}

Voxel::Voxel (const float x, const float y, const float z,
   const float wX, const float wY, const float wZ)
    : Voxel()
{
  set(x,y,z,wX,wY,wZ);
}

void Voxel::set(const float x, const float y, const float z,
   const float wX, const float wY, const float wZ)
{
    x_ = x;
    y_ = y;
    z_ = z;
    wx_ = wX;
    wy_ = wY;
    wz_ = wZ;
}

float Voxel::x() const
{
    return x_;
}
float Voxel::y() const
{
    return y_;
}
float Voxel::z() const
{
    return z_;
}
float Voxel::x_dim() const
{
    return wx_;
}
float Voxel::y_dim() const
{
    return wy_;
}
float Voxel::z_dim() const
{
    return wz_;
}

double
Voxel::volume() const
{
    return double(wx_)*double(wy_)*double(wz_);
}

void Voxel::set(const cv::Mat& origin, const cv::Mat& dims)
{
    set(origin.at<float>(0), origin.at<float>(1), origin.at<float>(2),
        dims.at<float>(0), dims.at<float>(1), dims.at<float>(2));
}

cv::Mat
Voxel::origin() const
{
    cv::Mat origin(3, 1, CV_32FC1);
    origin.at<float>(0) = x_;
    origin.at<float>(1) = y_;
    origin.at<float>(2) = z_;
    return origin;
}

cv::Mat
Voxel::dims() const
{
    cv::Mat dims(3, 1, CV_32FC1);
    dims.at<float>(0) = wx_;
    dims.at<float>(1) = wy_;
    dims.at<float>(2) = wz_;
    return dims;
}

std::ostream&
operator<<(std::ostream& out, const Voxel& voxel)
{
  out << voxel.x() << ':' << voxel.y() << ':' << voxel.z() << ':'
      << voxel.x_dim() << ':' << voxel.y_dim() << ':' << voxel.z_dim();
  return out;
}

std::istream&
operator>>(std::istream& in, Voxel& voxel)
{
  float x,y,z,xdim,ydim,zdim;
  char sep;
  in >> x >> sep >> y >> sep >> z >>sep
          >> xdim >> sep >> ydim >> sep >> zdim ;
  if (in)
    voxel.set(x, y, z, xdim, ydim, zdim);
  return in;
}

} //namespace fsiv
