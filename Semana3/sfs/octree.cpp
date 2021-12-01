#include "octree.hpp"

namespace fsiv {

Octant::Octant(const Voxel& v, OctantState state):
    voxel_(v), state_(state)
{}

bool
Octant::is_empty() const
{
    return voxel_.volume()==0.0;
}
const Voxel&
Octant::voxel() const
{
    return voxel_;
}

OctantState
Octant::state() const
{
    return state_;
}

void
Octant::set_state(OctantState new_st)
{
    state_ = new_st;
}

bool
Octant::is_leaf() const
{
    return children_.size()==0;
}

const Octant&
Octant::child(size_t i) const
{
    CV_Assert(!is_leaf() && i<8);
    return children_[i];
}

Octant&
Octant::child(size_t i)
{
    CV_Assert(!is_leaf() && i<8);
    return children_[i];
}

void
Octant::split()
{
    CV_Assert(!is_empty());
    CV_Assert(is_leaf());
    cv::Mat child_dims = 0.5*voxel().dims();
    cv::Mat child_off = cv::Mat(3,1, CV_32FC1);
    cv::Mat child_origin;
    for(int x = 0; x<2; ++x)
    {
        child_off.at<float>(0) = x;
        for(int y = 0; y<2; ++y)
        {
            child_off.at<float>(1) = y;
            for(int z = 0; z<2; ++z)
            {
                child_off.at<float>(2) = z;
                child_origin = voxel().origin() + child_dims.mul(child_off);
                children_.push_back(Octant(Voxel(child_origin, child_dims), BLACK));
            }
        }
    }

}

Octree::Octree():
    root_(nullptr)
{
    CV_Assert(is_empty());
}

bool
Octree::is_empty() const
{
    return root_==nullptr;
}

void
Octree::set_root(const Octant& root)
{
    root_=std::make_shared<Octant>(root);
}

const Octant&
Octree::root() const
{
    CV_Assert(!is_empty());
    return *root_;
}

Octant&
Octree::root()
{
    CV_Assert(!is_empty());
    return *root_;
}

static std::ostream&
operator<<(std::ostream& out, const Octant& oct)
{
    out << oct.state() << ' ';
    if (oct.state()==GREY)
        for (size_t c=0; c<8; c++)
            out << oct.child(c);
    return out;
}

static std::istream&
operator>>(std::istream& in, Octant& oct)
{
    int state;
    in >> state;
    if (in)
    {
        oct.set_state(static_cast<OctantState>(state));
        if (oct.state()==GREY)
        {
            oct.split();
            for (size_t c=0; c<8; c++)
                in >> oct.child(c);
        }
    }
    return in;
}

static std::string octree_signature="octree";

std::ostream&
operator << (std::ostream& out, const Octree& octree)
{
    out << octree_signature << std::endl;
    if (octree.is_empty())
        out << Voxel();
    else
        out << octree.root().voxel() << ' ' << octree.root();
    return out;
}

std::istream&
operator >> (std::istream& in, Octree& octree)
{
    std::string signature;
    in >> signature;
    if (in && signature==octree_signature)
    {
        Voxel voxel;
        in >> voxel;
        if (voxel.volume()>0.0)
        {
            Octant root(voxel);
            in >> root;
            octree.set_root(root);
        }
    }
    return in;
}

static void
save_as_cubes_WRML (std::ostream& out, const Octant& oct,
                    const cv::Scalar & color)
{
    if (oct.state()==BLACK)
    {
        cv::Mat pos3d=oct.voxel().center();
        cv::Mat dims =oct.voxel().dims();
        out << "Transform {\n";
        out << "  translation " <<  pos3d.at<float>(0) << ' ' << pos3d.at<float>(1) << ' ' << pos3d.at<float>(2) << std::endl;
        out << "  children Shape {\n";
        out << "     appearance Appearance  { material Material { diffuseColor " << color[0]/255.0 << ' ' << color[1]/255.0 << ' ' << color[2]/255.0 << "} }\n";
        out << "     geometry Box { size " << oct.voxel().x_dim() << ' ' << oct.voxel().y_dim() << ' ' << oct.voxel().z_dim() << " }\n";
        out << "  }\n";
        out << "}\n";
    }
    else if (oct.state()==GREY && !oct.is_leaf())
        for (size_t c=0; c<8; ++c)
            save_as_cubes_WRML(out, oct.child(c), color);
}

void save_as_cubes_WRML (std::ostream& out, const Octree& oct,
            const cv::Scalar & color)
{
    out << "#VRML V2.0 utf8\n";
    if (!oct.is_empty())
        save_as_cubes_WRML(out, oct.root(), color);
}


}
