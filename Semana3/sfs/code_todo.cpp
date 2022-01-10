#include <cmath>
#include "sfs.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#ifndef NDEBUG
#include <opencv2/highgui.hpp>
#endif

namespace fsiv
{

#ifndef NDEBUG
    int DebugLevel_ = 0;
#endif

    cv::Mat
    Voxel::center() const
    {
        cv::Mat center;
        //TODO
        //Supongo que cada fila es una de las coordenadas x y z
        center.push_back(x_ + wx_ / 2);
        center.push_back(y_ + wy_ / 2);
        center.push_back(z_ + wz_ / 2);
        //
        CV_Assert(center.rows = 3 && center.cols == 1 && center.type() == CV_32FC1);
        return center;
    }

    cv::Mat Voxel::vertices() const
    {
        cv::Mat ret = cv::Mat::zeros(3, 8, CV_32FC1);
        //TODO
        // for (int x = 0; x < ret.cols; x++)
        // {
        //     ret.at<float>(0,x) =
        //     ret.at<float>(1,x) =
        //     ret.at<float>(2,x) =
        // }
        //
        ret.at<float>(0, 0) = x_;
        ret.at<float>(1, 0) = y_;
        ret.at<float>(2, 0) = z_;

        ret.at<float>(0, 1) = x_;
        ret.at<float>(1, 1) = y_ + wy_;
        ret.at<float>(2, 1) = z_;

        ret.at<float>(0, 2) = x_;
        ret.at<float>(1, 2) = y_;
        ret.at<float>(2, 2) = z_ + wz_;

        ret.at<float>(0, 3) = x_;
        ret.at<float>(1, 3) = y_ + wy_;
        ret.at<float>(2, 3) = z_ + wz_;

        ret.at<float>(0, 4) = x_ + wx_;
        ret.at<float>(1, 4) = y_;
        ret.at<float>(2, 4) = z_;

        ret.at<float>(0, 5) = x_ + wx_;
        ret.at<float>(1, 5) = y_ + wy_;
        ret.at<float>(2, 5) = z_;

        ret.at<float>(0, 6) = x_ + wx_;
        ret.at<float>(1, 6) = y_;
        ret.at<float>(2, 6) = z_ + wz_;

        ret.at<float>(0, 7) = x_ + wx_;
        ret.at<float>(1, 7) = y_ + wy_;
        ret.at<float>(2, 7) = z_ + wz_;

        CV_Assert(ret.type() == CV_32FC1 && ret.rows == 3 && ret.cols == 8);
        return ret;
    }

    // Un voxelset va a ser un conjunto de voxels con 3 punteros, pero vamos a representar solo si están vacíos (0) o llenos(1)

    void
    VoxelSet::reset(const Voxel &bc, const float vsize,
                    const bool init_occ_state)
    {
        //TODO
        // División entera
        _x_size = bc.x_dim() / vsize;
        _y_size = bc.y_dim() / vsize;
        _z_size = bc.z_dim() / vsize;
        _xy_size = _x_size * _y_size;

        _vsize = vsize;

        if (init_occ_state)
            _occupancy_map = cv::Mat::ones(1, _x_size * _y_size * _z_size, CV_8UC1);
        else
            _occupancy_map = cv::Mat::zeros(1, _x_size * _y_size * _z_size, CV_8UC1);

        //
        CV_Assert(size() == x_size() * y_size() * z_size());
        CV_Assert(_occupancy_map.type() == CV_8UC1);
        CV_Assert(_occupancy_map.rows == 1 && _occupancy_map.cols == size());
    }

    size_t
    VoxelSet::xyz2index(const size_t x, const size_t y, const size_t z) const
    {
        CV_Assert(x < x_size() && y < y_size() && z < z_size());
        size_t idx = 0;
        //TODO

        idx = z * _xy_size + y * _x_size + x;
        // idx = z * x * _xy_size + y * x * _x_size + x;

        //
        CV_Assert(idx < size());
        return idx;
    }

    void
    VoxelSet::index2xyz(const size_t index, size_t &x, size_t &y, size_t &z) const
    {
        CV_Assert(index < size());
        //TODO
        z = index / _xy_size;
        size_t resto = index % _xy_size;
        y = resto / _x_size;
        x = resto % _x_size;

        //
        CV_Assert(x < x_size());
        CV_Assert(y < y_size());
        CV_Assert(z < z_size());
    }

    bool
    VoxelSet::is_external(const size_t x, const size_t y, const size_t z) const
    {
        CV_Assert(x < x_size() && y < y_size() && z < z_size());
        bool is_external = false;
        //TODO
        //Remember: A voxel is considered external if it is in the limits
        //of the voxelset or any of its neighbors voxels is not occupied.
        //Vamos, que si esta en la superficie de la figura

        //Cada voxel tiene 26 vecinos

        for (int i = 1; i <= 18; i++)
        {
            // En primer lugar será external si alguna de sus x, y, z valen 0, ya que esto indica que esta en una capa exterior
            if (x == 0 || y == 0 || z == 0)
            {
                is_external = true;
                break; // Si le falta un vecino ya es externo, no hay que seguir comprobando más
            }
            // Sera external si en las 18 posiciones anteriores y posteriores en occupancy no hay voxel (es 0)
            if (this->_occupancy_map.at<uchar>(xyz2index(x, y, z) + i) == 0 || this->_occupancy_map.at<uchar>(xyz2index(x, y, z) - i) == 0)
            {
                is_external = true;
                break; // Si le falta un vecino ya es externo, no hay que seguir comprobando más
            }
        }

        //
        return is_external;
    }

    cv::Mat
    CameraParameters::rotation_matrix() const
    {
        cv::Mat rotmat;
        //TODO
        //Hint: use the cv::Rodrigues function.
        cv::Rodrigues(_rotation_vector, rotmat);
        //
        CV_Assert(rotmat.rows == 3 && rotmat.cols == 3);
        return rotmat;
    }

    View::View(const std::string id, const cv::Mat &fg_img, const CameraParameters &cparams)
    {
        CV_Assert(fg_img.type() == CV_8UC1);
        _id = id;
        _foreground = fg_img.clone();
        _cparams = cparams;
        _view = fg_img.clone();
        //TODO
        //Compute the integral image from _fg_img.
        // cv::Mat integral_img = fg_img.clone();
        //First: we need use the fg_img with values 0|1.
        //Si es mayor que 0 ponemos 1, ya que aunque sea gris forma parte de la silueta
        // for (size_t y = 0; y<fg_img.rows;y++){
        //     for (size_t x = 0; x<fg_img.rows;x++){
        //         if(fg_img.at<uchar>(y,x) > 0){
        //             integral_img.at<uchar>(y,x) = 1;
        //         }
        //     }
        // }

        // //Second: use cv::integral.
        // cv::integral(integral_img,integral_img);

        // Más corto
        _foreground = (_foreground > 0) / 255;

        // std::cout<<_foreground<<std::endl;

        cv::integral(_foreground, _iimg_fg);

        // std::cout<<"integral_size: "<<_iimg_fg.size()<<std::endl;
        // std::cout<<"fgimg_size: "<<fg_img.size()<<std::endl;

        //
        CV_Assert(iimg().type() == CV_32S);
    }

    cv::Mat View::project_points(const cv::Mat &_3dPoints) const
    {
        cv::Mat _2dPoints;
        //TODO
        //Hint: use the cv::projectPoints() function.
        cv::projectPoints(_3dPoints, _cparams.rotation_vector(),
                          _cparams.translation_vector(),
                          _cparams.camera_matrix(),
                          _cparams.distortion_coeffs(),
                          _2dPoints);
        //
        CV_Assert(_2dPoints.channels() == 2);
        return _2dPoints;
    }
    cv::Rect
    View::compute_bounding_box(const cv::Mat &_3dPoints) const
    {
        cv::Rect bbox;
        //TODO
        //Remember: Find the bounding box that circumscribes the projections of the
        //3D points in the image plane.
        //The bounding box must be clipped using regarding image dimensions.
        //For that, use the overloaded operator '&' for the cv::Rect class.

        // Proyecta los 8 vertices en 2d
        cv::Mat points_2d = this->project_points(_3dPoints);

        cv::Point min_x, max_x, min_y, max_y;
        double min_x_value, max_x_value, min_y_value, max_y_value;
        cv::Mat col_x, col_y;

        for (int i = 0; i < points_2d.rows; i++)
        {
            col_x.push_back(points_2d.at<float>(i, 0));
            col_y.push_back(points_2d.at<float>(i, 1));
        }

        // std::cout << "Puntos 3d:\n"
        //           << _3dPoints << std::endl;
        // std::cout << "Puntos 3d size:" << _3dPoints.size() << std::endl;
        // std::cout << "Puntos 2d:\n"
        //           << points_2d << std::endl;
        // std::cout << "Puntos 2d size:" << points_2d.size() << std::endl;
        // std::cout << "Puntos 2d (0,0):" << points_2d.at<float>(0, 0) << std::endl;

        // std::cout<<"Col_x: "<<cv::Mat(points_2d,cv::Range(1,7),cv::Range(1,1))<<std::endl;
        // std::cout<<"Col_y: "<<points_2d(cv::Range())<<std::endl;
        // std::cout << "Col_x: " << col_x << std::endl;
        // std::cout << "Col_y: " << col_y << std::endl;
        cv::minMaxLoc(col_x, &min_x_value, &max_x_value, 0, 0);
        cv::minMaxLoc(col_y, &min_y_value, &max_y_value, 0, 0);

        bbox = cv::Rect(max_x_value, max_y_value, min_x_value, min_y_value);
        //bbox = cv::boundingRect(points_2d);

        // Para que el rectángulo no se salga de lam imagen
        bbox = bbox & cv::Rect(0, 0, this->_view.cols, this->_view.rows);

        // RectA & RectB devuelve la intersección de ambos Rect
        // bbox = _3dPoints
        //

        // std::cout<<bbox<<std::endl;

        return bbox;
    }

    int
    View::compute_occupied_area(const cv::Rect &bbox) const
    {
        CV_Assert(bbox.area() >= 0);
        int area = 0;


        // std::cout << iimg() << std::endl;
        // std::cout << iimg().at<uchar>(0, 0) << std::endl;

        int tl = this->iimg().at<uchar>((bbox.y), (bbox.x));
        int tr = this->iimg().at<uchar>((bbox.y), (bbox.x + bbox.width + 1));
        int bl = this->iimg().at<uchar>((bbox.y + bbox.height + 1), (bbox.x));
        int br = this->iimg().at<uchar>((bbox.y + bbox.height + 1), (bbox.x + bbox.width + 1));

        area = br - bl - tr + tl;

        //TODO
        //Hint: use the integral image of the foreground to do this with O(1).
        //See documentation of cv::integral
        // Número de pixeles ocupados de TODA la imagen, queremos solamente los de dentro del bbox
        // area = this->iimg().at<uchar>(this->iimg().rows - 1, this->iimg().cols - 1);
        //
        return area;
    }

    static bool
    voxelset_projection_test(const Voxel &voxel, std::vector<View> const &views,
                             const float OAR_th)
    {
        bool is_occupied = true;
        //TODO:
        //Remember: for each view test if the projected bbox has enough foreground area.
        //Not take into account a view if the projection is out of the image frame (bbox.area()==0).
        for (size_t i = 0; i < views.size(); i++)
        {
            // std::cout << "Vertices:\n"
            //           << voxel.vertices() << std::endl;
            cv::Rect bbox = views[i].compute_bounding_box(voxel.vertices());

            if (views[i].compute_occupied_area(bbox) <= 0)
            {
                return false;
            }
        }

        //
        return is_occupied;
    }

    void
    compute_visual_hull(std::vector<View> const &views, VoxelSet &vs,
                        const Voxel &scene, float vsize, const float OAR_th)
    {
        vs.reset(scene, vsize);
        //TODO
        // std::cout << vs.size() << std::endl;
        //Apply the SFS algorithm.
        //Do a projection test for each voxel.
        //Hint: use "#pragma omp parallel for" to parallelize the loop.
#pragma omp parallel for
        for (size_t voxel_idx = 0; voxel_idx < vs.size(); ++voxel_idx)
        {
            // std::cout << "VOXEL:" << vs.voxel(voxel_idx) << std::endl;
            bool is_occupied = voxelset_projection_test(vs.voxel(voxel_idx),
                                                        views, OAR_th);
            vs.set_occupancy(voxel_idx, is_occupied);
        }
        //
    }

    static OctantState
    octree_projection_test(const Voxel &voxel, std::vector<View> const &views,
                           const size_t level, const size_t max_levels,
                           const size_t max_errors, const float OAR_th)
    {
        OctantState st = BLACK;
        //TODO
        //Aply the Projection Test for each view.
        //Remeber: the inner nodes have three states:
        //       WHITE (OAR==0 && errors<max_errors),
        //           GREY((OAR==0 && errors<max_errors)||0<OAR<1), BLACK(OAR==1).
        //while leaf nodes (at the last level) only have two states:
        //       WHITE (OAR<OAR_th), BLACK(OAR>=OAR_th).

        //
        return st;
    }

    static void
    process_octant(Octant &oct, std::vector<View> const &views,
                   const size_t level, const size_t max_levels,
                   const size_t max_errors, const float OAR_th)
    {
        //TODO
        //Apply the SFS algorithm on this octant.
        //First compute the its state using the projection test.
        //Second if the state is GREY, split and do recursion to go down in the tree.
        //Remenber to actualize level var in the recursion(level+1).

        //no usar max errors(si no tenemos tiempo). Si todas dicen que si, esta ocupado, si solo una dice que esta vacio, esta vacio
        //max_error sería el número de vistas que permito que digan que no está ocupado y considero que sí
        //
    }

    void
    compute_visual_hull(std::vector<View> const &views, Octree &octree,
                        const Voxel &scene, size_t max_levels,
                        size_t max_errors, const float OAR_th)
    {
        CV_Assert(max_errors <= views.size());
        //TODO
        //Apply the SFS algorithm.
        //First, Create a root octant and processed.
        //Second, set the octant as the root node of the octree.

        //
    }

} //namespace fsiv
