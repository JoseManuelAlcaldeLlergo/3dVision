#pragma once
#include <iostream>
#include <opencv2/core.hpp>

#include "voxel.hpp"

namespace fsiv
{


/**
 * @brief The Voxelset class.
 *
 * Models a discretized volumen of 3D space where each piece is a voxel.
 */
class VoxelSet
{
public:

    VoxelSet ();
    VoxelSet(const Voxel& bc, const float vsize, const bool init_occ_state=true);

    bool empty();

    /**
     * @brief set voxel set attributes.
     * @arg[in] bc define the 3D volumen represented.
     * @arg[in] vsize define the size of the voxel in wcs units.
     * @arg[in] init_occ_state is the initial state of occupancy for the voxels.
     */
    void reset(const Voxel& bc, const float vsize, const bool init_occ_state=true);

    /** @brief convert from x,y,z indexes to 1D index. **/
    size_t xyz2index (const size_t x, const size_t y, const size_t z) const;

    /** @brief convert from 1D index to x,y,z indexes. **/
    void index2xyz (const size_t index, size_t& x, size_t& y, size_t& z) const;

    /** @brief Get a voxel.*/
    Voxel voxel(const size_t idx) const;

    /** @brief Get a voxel.*/
    Voxel voxel(const size_t x, const size_t y, const size_t z) const;

    /** @brief get the occupancy value. **/
    bool occupancy(const size_t idx) const;

    /** @brief get the occupancy value. **/
    bool occupancy(const size_t x, const size_t y, const size_t z) const;

    /** @brief set the occupancy value. **/
    void set_occupancy(const size_t idx, const bool new_v);

    /** @brief set the occupancy value. **/
    void set_occupancy(const size_t x, const size_t y, const size_t z, const bool new_v);

    /**
     * @brief test if a voxel is external.
     * A voxel is considered external if it is in the limits of the voxelset or
     * any of its neighbors voxels is not occupied.
     */
    bool is_external(size_t idx) const;

    /**
     * @brief test if a voxel is external.
     * A voxel is considered external if it is in the limits of the voxelset or
     * any of its neighbors voxels is not occupied.
     */
    bool is_external(const size_t x, const size_t y, const size_t z) const;

    /** @brief get the discritized size on X axis. **/
    size_t x_size () const;

    /** @brief get the discritized size on Y axis. **/
    size_t y_size () const;

    /** @brief get the discritized size on Z axis. **/
    size_t z_size () const;

    /** @brief get the discritized size as 1D array. **/
    size_t size () const;

    /** @brief get the side voxel size in WCS units. **/
    float vsize() const;    

    /** @brief get the 3D space volumen represented by the voxel set. **/
    const Voxel& bounding_cuve() const;

    /** @brief get raw data of occupancy **/
    const cv::uint8_t* data() const;

    /** @brief get raw data of occupancy **/
    cv::uint8_t* data();

private:

    Voxel _bcuve; //el volumen de la habitación a discrtizar (tb es un voxel que vamos a dividir en voxeles más pequeños)
    float _vsize;
    size_t _x_size;
    size_t _y_size;
    size_t _z_size;
    size_t _xy_size;
    cv::Mat _occupancy_map; //Array de 3 dimensiones
};

/** @brief Save a voxelset from a file. **/
std::ostream& operator<<(std::ostream& out, const VoxelSet& vs);

/** @brief Load a voxelset from a file. **/
std::istream& operator>>(std::istream& in, VoxelSet& vs);

/** @brief Save a voxelset as a point cloud of occupied voxel centers in vrml 2.0 format
 * For each occupied voxel the 3D center point is saved.
**/
void save_as_pointcloud_WRML (std::ostream& out, const VoxelSet& vs,
            const cv::Scalar & color=cv::Scalar(255, 255, 255));

/** @brief Save a voxelset as set of 3d cubes in vrml 2.0 format
 * \warning only external cubes are saved. **/
void save_as_cubes_WRML (std::ostream& out, const VoxelSet& vs,
            const cv::Scalar & color=cv::Scalar(255, 255, 255));


} //namespacde fsiv
