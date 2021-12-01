#include <zlib.h>
#include <valarray>
#include "voxelset.hpp"

namespace fsiv {

VoxelSet::VoxelSet ()
    : _bcuve(), _vsize(0.0),
      _x_size(0), _y_size(0), _z_size(0), _xy_size(0)
{
    CV_Assert(empty());
}

VoxelSet::VoxelSet(const Voxel& bc, const float vsize, const bool init_occ_state)
{
  reset(bc, vsize, init_occ_state);
}

bool
VoxelSet::empty()
{
    return _occupancy_map.empty();
}

Voxel
VoxelSet::voxel(const size_t x, const size_t y, const size_t z) const
{
    CV_Assert (x<x_size() && y<y_size() && z<z_size());
    return Voxel(_bcuve.origin().at<float>(0)+_vsize*x,
                 _bcuve.origin().at<float>(1)+_vsize*y,
                 _bcuve.origin().at<float>(2)+_vsize*z,
                 _vsize, _vsize, _vsize);
}

Voxel
VoxelSet::voxel(const size_t idx) const
{
    CV_Assert(idx < size());
    size_t x,y,z;
    index2xyz(idx, x, y, z);
    return voxel(x, y, z);
}

bool
VoxelSet::occupancy(const size_t idx) const
{
    CV_Assert(idx < size());
    return _occupancy_map.at<uchar>(idx);
}

bool
VoxelSet::occupancy(const size_t x, const size_t y, const size_t z) const
{
  CV_Assert (x<x_size() && y<y_size() && z<z_size());
  return occupancy(xyz2index(x, y, z));
}

void
VoxelSet::set_occupancy(const size_t idx, const bool new_v)
{
    CV_Assert(idx < size());
    _occupancy_map.at<cv::uint8_t>(idx)=new_v;
}

void
VoxelSet::set_occupancy(const size_t x, const size_t y, const size_t z,
                        const bool new_v)
{
  CV_Assert (x<x_size() && y<y_size() && z<z_size());
  return set_occupancy(xyz2index(x, y, z), new_v);
}

size_t
VoxelSet::size () const
{
  return _occupancy_map.cols;
}

size_t
VoxelSet::x_size () const
{
  return _x_size;
}

size_t
VoxelSet::y_size () const
{
  return _y_size;
}

size_t
VoxelSet::z_size () const
{
  return _z_size;
}

float
VoxelSet::vsize() const
{
  return _vsize;
}

const Voxel& VoxelSet::bounding_cuve() const
{
  return _bcuve;
}

cv::uint8_t*
VoxelSet::data()
{
    return _occupancy_map.data;
}

const cv::uint8_t*
VoxelSet::data() const
{
    return _occupancy_map.data;
}

static std::string voxelset_signature = "voxelset";

std::ostream&
operator<<(std::ostream& out, const VoxelSet& vs)
{
  out.setf(std::ios::scientific);
  out << voxelset_signature << std::endl;
  out << "vsize = " << vs.vsize() << std::endl;
  out << "bcuve = " << vs.bounding_cuve() << std::endl;
  out << "xsize = " << vs.x_size()<< std::endl;
  out << "ysize = " << vs.y_size()<< std::endl;
  out << "zsize = " << vs.z_size()<< std::endl;
  out.unsetf(std::ios::scientific);
  uLong sourceLen = vs.size();
  uLongf destLen = compressBound(sourceLen);
  std::valarray<Bytef> dest (destLen+1);
  dest[0]='#';
  int error;
  if ((error=compress(reinterpret_cast<Bytef*>(&(dest[1])), &destLen,
                      reinterpret_cast<const Bytef *>(vs.data()), sourceLen))!=Z_OK)
    throw std::runtime_error("ZLIB error");
  out << "BinarySize = " << destLen << std::endl;
  out.write(reinterpret_cast<const char *>(&dest[0]), destLen+1);
  return out;
}

std::istream&
operator>>(std::istream& in, VoxelSet& vs)
{
    std::string signature;
    in >> signature;
    std::string key, sep;
    if (!in && signature!=voxelset_signature)
        throw std::runtime_error("Wrong Input Format: signature.");
    float vsize;
    in >> key >> sep >> vsize;
    if (!in || key!="vsize")
        throw std::runtime_error("Wrong Input Format: vsize");
    Voxel bcuve;
    in >> key >> sep >> bcuve;
    if (!in || key != "bcuve")
        throw std::runtime_error("Wrong Input Format: bounding cuve");
    unsigned xsize;
    in >> key >> sep >> xsize;
    if (!in || key!="xsize")
        throw std::runtime_error("Wrong Input Format: xsize");
    unsigned ysize;
    in >> key >> sep >> ysize;
    if (!in || key!="ysize")
        throw std::runtime_error("Wrong Input Format: ysize");
    unsigned zsize;
    in >> key >> sep >> zsize;
    if (!in || key!="zsize")
        throw std::runtime_error("Wrong Input Format: zsize");

    uLong sourceLen;
    in >> key >> sep >> sourceLen;
    if (!in || key!="BinarySize")
        throw std::runtime_error("Wrong Input Format: binary size");

    //find the flag signs the binary data.
    while (in && in.peek()!='#')
        in.get();
    in.get();//pull out the flag.

    std::valarray<Bytef> source (sourceLen);
    in.read(reinterpret_cast<char *>(&source[0]), sourceLen);
    if (!in)
        throw std::runtime_error("Wrong Input Format: raw data");

    vs.reset(bcuve, vsize);
    if (vs.x_size()!=xsize ||
            vs.y_size()!=ysize ||
            vs.z_size()!=zsize)
        throw std::runtime_error("Wrong Input Format: wrong dimensions.");
    uLongf destLen = sizeof(float)*xsize*ysize*zsize;

    int error;
    if ((error=uncompress(reinterpret_cast<Bytef*>(vs.data()), &destLen,
                          reinterpret_cast<const Bytef*>(&source[0]),
                          sourceLen))!=Z_OK)
        throw std::runtime_error("Wrong Input Format: ZLIB uncompress error.");
    return in;
}

void
save_as_pointcloud_WRML (std::ostream& out, const VoxelSet& vs,
              const cv::Scalar & _color)
{
  out << "#VRML V2.0 utf8\n";
  out << "Transform\n";
  out << "{\n";
  out << "  children\n";
  out << "  Shape\n";
  out << "  {\n";
  out << "    appearance Appearance\n";
  out << "    {\n";
  out << "      material Material\n";
  out << "      {\n";
  out << "       emissiveColor " << _color[0]/255.0 << ' ' << _color[1]/255.0 << ' ' << _color[2]/255.0 << "\n";
  out << "      }\n";
  out << "    }\n";
  out << "    geometry PointSet\n";
  out << "    {\n";
  out << "      coord Coordinate\n";
  out << "      {\n";
  out << "        point\n";
  out << "          [\n";
  for (size_t v=0; v<vs.size() ;++v)
  {
    if ( vs.occupancy(v) )
    {
      cv::Mat pos3d=vs.voxel(v).center();
      out << pos3d.at<float>(0) << ' ' << pos3d.at<float>(1) << ' ' << pos3d.at<float>(2) << std::endl;
    }
  }
  out << "         ]\n";
  out << "      }\n";
  out << "    }\n";
  out << "  }\n";
  out << "}\n";
}

bool
VoxelSet::is_external(const size_t idx) const
{
    CV_Assert (idx<size());
    size_t x,y,z;
    index2xyz(idx, x, y, z);
    return is_external(x, y, z);
}


void save_as_cubes_WRML (std::ostream& out, const VoxelSet& vs,
            const cv::Scalar & color)
{
    out << "#VRML V2.0 utf8\n";
    for (size_t v=0; v<vs.size() ;++v)
    {
      if ( vs.occupancy(v) && vs.is_external(v))
      {
        cv::Mat pos3d=vs.voxel(v).center();
        out << "Transform {\n";
        out << "  translation " <<  pos3d.at<float>(0) << ' ' << pos3d.at<float>(1) << ' ' << pos3d.at<float>(2) << std::endl;
        out << "  children Shape {\n";
        out << "     appearance Appearance  { material Material { diffuseColor " << color[0]/255.0 << ' ' << color[1]/255.0 << ' ' << color[2]/255.0 << "} }\n";
        out << "     geometry Box { size " << vs.vsize() << ' ' << vs.vsize() << ' ' << vs.vsize() << " }\n";
        out << "  }\n";
        out << "}\n";
      }
    }
}



} //namespace fsiv
