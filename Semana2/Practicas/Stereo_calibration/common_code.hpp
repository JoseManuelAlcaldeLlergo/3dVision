#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

/**
 * @brief Generate a 3d point vector with the inner corners of a calibration board.
 * @param board_size is the inner points board geometry (cols x rows).
 * @param square_size is the size of the squares.
 * @return a vector of 3d points with the corners.
 * @post ret_v.size()==(cols*rows)
 */
std::vector<cv::Point3f> generate_3d_calibration_points(const cv::Size& board_size,
                                                        float square_size);

/**
 * @brief Find a calibration chessboard and compute the refined coordinates of the inner corners.
 * @param img is the image where finding out.
 * @param board_size is the inners board points geometry.
 * @param[out] corner_points save the refined corner coordinates if the board was found.
 * @param wname is its not nullptr, it is the window's name use to show the detected corners.
 * @return true if the board was found.
 * @pre img.type()==CV_8UC3
 * @warning A keyboard press is waited when the image is shown to continue.
 */
bool find_chessboard_corners(const cv::Mat& img, const cv::Size &board_size,
                                  std::vector<cv::Point2f>& corner_points,
                                  const char * wname=nullptr);


/**
 * @brief Save the calibration parameters in a file.
 *
 * @param[in|out] fs is a file storage object to write the data.
 * @param[in] camera_size is the camera geometry in pixels.
 * @param[in] error is the calibration error.
 * @param[in] camera_matrix_left is the camera matrix of the left image.
 * @param[in] camera_matrix_right is the camera matrix of the right image.
 * @param[in] dist_coeffs_left are the distortion coefficients of the left image.
 * @param[in] rvec is the rotation vector.
 * @param[in] tvec is the translation vector.
 * @param[in] E is the Essential matrix
 * @param[in] F is the Fundamental matrix.
 * @pre fs.isOpened()
 * @post fs.isOpened()
 */
void save_calibration_parameters(cv::FileStorage& fs,
                                const cv::Size & camera_size,
                                float error,
                                const cv::Mat& camera_matrix_left,
                                const cv::Mat& camera_matrix_right,
                                const cv::Mat& dist_coeffs_left,
                                const cv::Mat& dist_coeffs_right,
                                 const cv::Mat& rvec,
                                 const cv::Mat& tvec,
                                 const cv::Mat& E,
                                 const cv::Mat& F);

/**
 * @brief Save the calibration parameters in a file.
 *
 * @param[in] s the path which have to be checked.
 */
bool IsPathExist(const std::string &s);