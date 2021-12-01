#pragma once

#include "voxel.hpp"
#include "voxelset.hpp"
#include "camera_parameters.hpp"
#include "view.hpp"
#include "octree.hpp"

namespace fsiv
{

#ifndef NDEBUG
extern int DebugLevel_;
#endif

/** @brief Compute a voxelset based visual hull from a group of views.
 * @param views is the set of views for the scene.
 * @param vs is the output voxelset.
 * @param scene is the 3D volumen to be analysed.
 * @param vsize specifies the size of a voxel in WCS units.
 * @param OAR_th specifies the minimum area rate to consider a full voxel.
 */
void compute_visual_hull(std::vector<View> const& views, VoxelSet& vs,
                         const Voxel& scene, float vsize,
                         const float OAR_th=0.5);

/**
 * @brief Compute a octree based visual hull from a group of views.
 * @param views is the set of views for the scene.
 * @param octree is the output octree.
 * @param scene is the 3D volumen to be analysed.
 * @param max_levels specifies the max height of the octree to be built.
 * @param max_errors specifies how many fouls are allowed before a voxel is considered empty.
 * @param OAR_th specifies the minimum area rate to consider a full voxel.
 */
void compute_visual_hull(std::vector<View> const& views, Octree& octree,
                         const Voxel& scene, size_t max_levels,
                         size_t max_erros=0, const float OAR_th=0.5);

} //namespacde fsiv
