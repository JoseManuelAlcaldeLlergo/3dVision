#pragma once
#include <iostream>
#include <opencv2/core.hpp>

#include "camera_parameters.hpp"

namespace fsiv
{

/** @brief Model a 2D view of a 3D scene.

  A view is formed by a image from a camera, optionally the foreground
  detected and the camera calibrations parameters.

  \see CameraParameters
*/
class View
{
public:

    /**
     * @brief Create a new view.
     * @param id is a identifier for this view.
     * @param fg_img is the foreground image (0|255).
     * @param cparams are the calibration parameters for the view.
     */
    View(const std::string id, const cv::Mat& fg_img, const CameraParameters& cparams);

    /*!\brief Get the view identificator.
    \return the view identificator.
  */
    const std::string&  id () const;

    /*!\brief Set a colored view image.
    \param[in] view is the new image.
  */
    void  set_colored_view (const cv::Mat& view);

    /*!\brief Get the view image.
    \return a reference to the view image.
  */
    const cv::Mat&  colored_view (void) const;

    /*!\brief Get the foreground image.
    \return a reference to the foreground image.
  */
    const cv::Mat& foreground (void) const;

    /*!\brief Get the integral image.*/
    const cv::Mat& iimg(void) const;

    /**
   * @brief Project 3D points onto the view.
   * @param _3dPoints are the coordinates to be projected.
   * @return the 2d coordinates.
   */
    cv::Mat project_points(const cv::Mat& _3dPoints) const;

    /*!\brief Given a sequence of 3D points, compute the projected 3D bounding box.
   * If the bbox area is zero means the points are projected out of the image frame.
   * \warning the bbox is clipped to the image frame.
   */
    cv::Rect compute_bounding_box(const cv::Mat& _3dPoints) const;

    /*!\brief Compute the number of foreground active pixels inside of a bbox */
    int compute_occupied_area(const cv::Rect& bbox) const;

private:
    std::string _id; /*!< the view identificator.*/
    cv::Mat _view; 	/*!< the view image.*/
    cv::Mat _foreground; 	/*!< the foreground image.*/
    cv::Mat _iimg_fg; /*!< the foreground integral image.*/
    CameraParameters _cparams; /*!< the camera parameters. */
};

} //namespacde fsiv
