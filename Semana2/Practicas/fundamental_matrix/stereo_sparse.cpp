// ./stereo_sparse ../reconstruction/m001.jpg ../stereo_calibration.yml  outpu.pcd

#include <iostream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d.hpp>

#include "common_code.hpp"

const cv::String keys =
    "{help h usage ? |      | print this message.}"
    "{@input        |<none>| path of the image.}"
    "{@calibration        |<none>| filename of the calibration file.}"
    "{@output        |<none>| PCD output file.}";

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;

    try
    {
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Stereo Calibration");
        if (parser.has("help"))
        {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        if (argc != 4)
        {
            std::cerr << "Se le deben pasar tres argumentos al programa:\n\t ./stereo_disparity stereo_image calibration.yml out.pcd" << std::endl;
            return EXIT_FAILURE;
        }

        cv::Mat cam_mat_left, cam_mat_right, dist_coef_left, dist_coef_right, R, T, E, F;
        StereoParams st_parameters;
        OnMouseParams mouse_parameters_or;
        OnMouseParams mouse_parameters_rect;
        std::string wname_or = "Original";
        std::string wname_rect = "Rectification";

        float error;
        cv::Size camera_size = cv::Size(0, 0);
        std::string img_name = parser.get<cv::String>("@input");
        std::string calibration_file = parser.get<cv::String>("@calibration");
        std::string output_file = parser.get<cv::String>("@output");

        if (!IsPathExist(img_name))
        {
            std::cerr << "No existe el la imagen <" << img_name << ">" << std::endl;
            return EXIT_FAILURE;
        }
        if (!IsPathExist(calibration_file))
        {
            std::cerr << "No existe el fichero de calibración <" << calibration_file << ">" << std::endl;
            return EXIT_FAILURE;
        }

        auto fs = cv::FileStorage();
        fs.open(calibration_file, cv::FileStorage::READ);
        load_calibration_parameters(fs, camera_size, error, st_parameters.mtxL, st_parameters.mtxR, st_parameters.distL, st_parameters.distR, st_parameters.Rot, st_parameters.Trns, st_parameters.Emat, st_parameters.Fmat);

        //Imagen original
        cv::Mat img = cv::imread(img_name);

        // // Dividimos la imagen en las dos que la componen
        cv::Mat img_left = img(cv::Range(0, img.rows), cv::Range(0, round(img.cols / 2)));
        cv::Mat img_right = img(cv::Range(0, img.rows), cv::Range(round(img.cols / 2), img.cols));

        rectifyStereoImages(st_parameters, img_left, img_right);

        //Imagen rectificada

        cv::Mat img_rect;
        cv::hconcat(img_left, img_right, img_rect);

        //Practica 4

        std::vector<cv::KeyPoint> keypoints_query, keypoints_train, keypoints_query_filtered, keypoints_train_filtered;
        cv::Mat descriptors_query, descriptors_train;
        std::vector<cv::DMatch> matches, filter_matches;
        cv::Mat matches_image, filter_matches_image;
        // std::vector<double> kp_distances;
        std::vector<cv::Point3f> points_3d;

        auto Detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 1e-4f);
        Detector->detectAndCompute(img_left, cv::Mat(), keypoints_query, descriptors_query);
        Detector->detectAndCompute(img_right, cv::Mat(), keypoints_train, descriptors_train);
        auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        matcher->match(descriptors_query, descriptors_train, matches, cv::Mat());

        std::cout << matches.size() << " matches" << std::endl;
        cv::drawMatches(img_left, keypoints_query, img_right, keypoints_train, matches, matches_image);

        cv::namedWindow("Matches", CV_WINDOW_NORMAL);
        cv::imshow("Matches", matches_image);
        cv::waitKey(0);
        // cv::imshow("DescQ", descriptors_query);
        // cv::waitKey(0);
        // cv::imshow("DescT", descriptors_train);

        // cv::waitKey(0);

        //Triangulation
        float B = sqrt((float)pow(st_parameters.Trns.at<double>(0, 0), 2) + (float)pow(st_parameters.Trns.at<double>(1, 0), 2) + (float)pow(st_parameters.Trns.at<double>(2, 0), 2));
        float f = (float)st_parameters.mtxL.at<double>(0, 0);
        float cx = (float)st_parameters.mtxL.at<double>(0, 2);
        float cy = (float)st_parameters.mtxL.at<double>(1, 2);
        float X, Y, Z;

        for (unsigned int i = 0; i < matches.size(); i++)
        {
            // If the distance between the y values of query and train keypoints which match are higher than 4 pixels we filter them
            if ((abs(keypoints_query[matches[i].queryIdx].pt.y - keypoints_train[matches[i].trainIdx].pt.y) < 4.0))
            {
                // The current size of the filtered matches vector will define the index of te new match
                int current_size = filter_matches.size();
                filter_matches.push_back(cv::DMatch(current_size, current_size, 0));
                keypoints_query_filtered.push_back(keypoints_query[matches[i].queryIdx]);
                keypoints_train_filtered.push_back(keypoints_train[matches[i].trainIdx]);
                //We are considering that they are in the same horizontal line
                // kp_distances.push_back(abs(keypoints_query[matches[i].queryIdx].pt.x) - keypoints_train[matches[i].trainIdx].pt.x);
                float disp = abs(keypoints_query[matches[i].queryIdx].pt.x - keypoints_train[matches[i].trainIdx].pt.x);
                if (disp > 10.0)
                {

                    Z = (B * f) / abs(keypoints_query[matches[i].queryIdx].pt.x - keypoints_train[matches[i].trainIdx].pt.x);
                    X = (keypoints_query[matches[i].queryIdx].pt.x - cx) * Z / f;
                    Y = (keypoints_query[matches[i].queryIdx].pt.y - cy) * Z / f;
                    points_3d.push_back(cv::Point3f(X, Y, Z));
                }
            }
        }
        std::cout << filter_matches.size() << std::endl;

        cv::drawMatches(img_left, keypoints_query_filtered, img_right, keypoints_train_filtered, filter_matches, filter_matches_image);

        cv::namedWindow("Filtered Matches", CV_WINDOW_NORMAL);
        cv::imshow("Filtered Matches", filter_matches_image);
        cv::waitKey(0);

        writeToPCD(output_file, points_3d);
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
