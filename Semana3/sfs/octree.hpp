#pragma once

#include <memory>
#include <vector>
#include "voxel.hpp"


namespace fsiv {


typedef enum {
    WHITE=0, //empty.
    GREY=1,  //unknown.
    BLACK=2  //occupied.
} OctantState;

/**
 * @brief The Octant class.
 * Models a node of the octree.
 */
class Octant
{
public:
    Octant(const Voxel& v=Voxel(), OctantState state=BLACK);
    bool is_empty() const;
    const Voxel& voxel() const;
    OctantState state() const;
    void set_state(OctantState new_state);
    void split();
    bool is_leaf() const;
    const Octant& child(size_t i) const;
    Octant& child(size_t i);

private:
    Voxel voxel_;
    OctantState state_;
    std::vector<Octant> children_;
};

/**
 * @brief The Octree class
 * Models a 8-ary tree splitting a 3D volumen in a recursive way.
 */
class Octree
{
public:
    Octree();    
    bool is_empty() const;
    void set_root(const Octant& root);
    const Octant& root() const;
    Octant& root();
private:
    std::shared_ptr<Octant> root_;

};

std::ostream& operator << (std::ostream& out, const Octree& octree);
std::istream& operator >> (std::istream& in, Octree& octree);
void save_as_cubes_WRML (std::ostream& out, const Octree& oct,
            const cv::Scalar & color=cv::Scalar(255, 255, 255));

} //namespace fsiv

