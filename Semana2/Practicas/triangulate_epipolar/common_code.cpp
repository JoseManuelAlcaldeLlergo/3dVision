#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <sys/stat.h>
#include <fstream>
#include <iostream>
#include "common_code.hpp"

CP readCameraParams(cv::FileStorage &fs)
{
    CV_Assert(fs.isOpened());

    CP cam_params;

    fs["camera_matrix"] >> cam_params.camera_matrix;
    fs["distortion_coefficients"] >> cam_params.dist_coefs;

    return cam_params;
}

void writeToPCD(std::string path, std::vector<cv::Point3f> points)
{
    std::ofstream file(path, std::ios::binary);
    if (!file)
        throw std::runtime_error("Could not ope  file:" + path);
    file << "# .PCD v.7 - Point Cloud Data file format" << std::endl
         << "VERSION .7" << std::endl;
    file << "FIELDS x y z " << std::endl
         << "SIZE 4 4 4 " << std::endl;
    file << "TYPE F F F " << std::endl
         << "COUNT 1 1 1 " << std::endl;
    file << "WIDTH " << points.size() << std::endl
         << "HEIGHT 1" << std::endl;
    file << "VIEWPOINT 0 0 0 1 0 0 0" << std::endl;
    file << "POINTS " << points.size() << std::endl;
    file << "DATA binary" << std::endl;
    file.write((char *)&points[0], sizeof(cv::Point3f) * points.size());
}

bool IsPathExist(const std::string &s)
{
    struct stat buffer;
    return (stat(s.c_str(), &buffer) == 0);
}

void __imshow(std::string wn, const cv::Mat &im, cv::Size ss)
{

    if (im.size() == ss)
        cv::imshow(wn, im);
    else
    {
        cv::Mat resized;
        cv::resize(im, resized, ss);
        cv::imshow(wn, resized);
    }
}
void showEpipolar(cv::Mat centralImage, cv::Mat otherImage, cv::Mat CamK, cv::Mat F)
{

    if (F.empty())
        return;

    struct CallBackInfo
    {
        cv::Mat imA, imB;
        cv::Mat CamK;
        cv::Mat F;
    };

    CallBackInfo *cbi = new CallBackInfo{centralImage, otherImage, CamK, F};

    cv::Mat Mixed(centralImage.rows, centralImage.cols * 2, CV_8UC3);
    if (centralImage.channels() == 1)
    {
        cv::cvtColor(centralImage, Mixed.colRange(0, centralImage.cols), cv::COLOR_GRAY2BGR);
        cv::cvtColor(otherImage, Mixed.colRange(centralImage.cols, 2 * centralImage.cols), cv::COLOR_GRAY2BGR);
    }
    else
    {
        centralImage.copyTo(Mixed.colRange(0, centralImage.cols));
        otherImage.copyTo(Mixed.colRange(centralImage.cols, 2 * centralImage.cols));
    }

    cv::namedWindow("Fundamental");
    __imshow("Fundamental", Mixed, cv::Size{1600, 600});

    cv::setMouseCallback(
        "Fundamental", [](int event, int x, int y, int flags, void *_cbi) -> void
        {
            if (x < 800)
            {
                CallBackInfo *cbi = (CallBackInfo *)_cbi;

                int sX = float(x) * float(cbi->imB.cols) / 800.;
                int sY = float(y) * (float(cbi->imB.rows) / 600.);

                //find epipolar line
                cv::Mat P = (cv::Mat_<double>(3, 1) << sX, sY, 1);
                cv::Mat L = cbi->F * P; //epipolar line
                double a = L.at<double>(0, 0);
                double b = L.at<double>(0, 1);
                double c = L.at<double>(0, 2);
                int x0 = 0;
                int y0 = (a * x0 + c) / -b;
                int x1 = cbi->imB.cols;
                int y1 = (a * x1 + c) / -b;
                cv::Mat imBcopy;
                cv::cvtColor(cbi->imB, imBcopy, cv::COLOR_GRAY2BGR);
                cv::line(imBcopy, cv::Point(x0, y0), cv::Point(x1, y1), {0, 0, 244}, 2);

                cv::Mat Mixed(cbi->imA.rows, cbi->imA.cols * 2, CV_8UC3);
                cv::cvtColor(cbi->imA, Mixed.colRange(0, cbi->imA.cols), cv::COLOR_GRAY2BGR);
                imBcopy.copyTo(Mixed.colRange(cbi->imA.cols, 2 * cbi->imA.cols));

                cv::namedWindow("Fundamental");
                __imshow("Fundamental", Mixed, cv::Size{1600, 600});
            }
            else
            {
            }
        },
        cbi);
}

cv::Mat removeDistortion(cv::Mat img, CP cam_params)
{
    cv::Mat und_img;
    cv::undistort(img, und_img, cam_params.camera_matrix, cam_params.dist_coefs);

    return und_img;
}

float epipolarLineSqDist(const cv::Point2f &kp1, const cv::Point2f &kp2, const cv::Mat &F)
{
    if (F.empty())
        return 0;
    // Epipolar line in second image l = x1'F = [a b c]
    auto a = (float)kp1.x * F.at<double>(0, 0) + kp1.y * F.at<double>(1, 0) + F.at<double>(2, 0);
    auto b = (float)kp1.x * F.at<double>(0, 1) + kp1.y * F.at<double>(1, 1) + F.at<double>(2, 1);
    auto den = a * a + b * b;
    if (den == 0)
        return std::numeric_limits<float>::max();
    auto c = (float)kp1.x * F.at<double>(0, 2) + kp1.y * F.at<double>(1, 2) + F.at<double>(2, 2);
    auto num = a * kp2.x + b * kp2.y + c;
    return num * num / den;
}

std::vector<cv::DMatch> KpMatch(std::vector<cv::KeyPoint> keypoints_query,
                                cv::Mat descriptors_query, std::vector<cv::KeyPoint> keypoints_train,
                                cv::Mat descriptors_train, cv::Mat im1, cv::Mat im2,
                                std::vector<cv::KeyPoint> &keypoints_query_filtered,
                                std::vector<cv::KeyPoint> &keypoints_train_filtered,
                                cv::Mat F = cv::Mat())
{

    std::vector<std::vector<cv::DMatch>> matches;
    std::vector<cv::DMatch> filt_matches;
    cv::Mat filter_matches_image;

    auto matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
    matcher->knnMatch(descriptors_query, descriptors_train, matches, 2);

    for (size_t i = 0; i < matches.size(); i++)
    {
        // std::cout<<epipolarLineSqDist(keypoints_query[matches[i][0].queryIdx].pt, keypoints_train[matches[i][0].trainIdx].pt, F)<<std::endl;
        // if ((matches[i][0].distance < 80) &&
        //     (abs(keypoints_query[matches[i][0].queryIdx].octave - keypoints_train[matches[i][0].trainIdx].octave) <= 2) &&
        //     matches[i][0].distance / matches[i][1].distance < 0.8 &&
        //     (epipolarLineSqDist(keypoints_train[matches[i][0].trainIdx].pt, keypoints_query[matches[i][0].queryIdx].pt, F) <= 4))
        if ((matches[i][0].distance < 80) &&
            (std::fabs(keypoints_query[matches[i][0].queryIdx].octave - keypoints_train[matches[i][0].trainIdx].octave) < 2.0) &&
            ((matches[i][0].distance / matches[i][1].distance) <= 0.8) &&
            (epipolarLineSqDist(keypoints_train[matches[i][0].trainIdx].pt, keypoints_query[matches[i][0].queryIdx].pt, F) <= 4.0))
        {
            // The current size of the filtered matches vector will define the index of te new match
            int current_size = static_cast<int>(keypoints_query_filtered.size());
            filt_matches.push_back(cv::DMatch(current_size, current_size, 0));
            keypoints_query_filtered.push_back(keypoints_query[matches[i][0].queryIdx]);
            keypoints_train_filtered.push_back(keypoints_train[matches[i][0].trainIdx]);
        }
    }

    // cv::drawMatches(im1, keypoints_query_filtered, im2, keypoints_train_filtered, filt_matches, filter_matches_image);

    // cv::namedWindow("Matches", CV_WINDOW_NORMAL);
    // cv::imshow("Matches", filter_matches_image);

    return filt_matches;
}

cv::Mat fundamental(cv::Mat im1, cv::Mat im2)
{
    cv::Mat F;
    std::vector<cv::KeyPoint> keypoints_query, keypoints_train, keypoints_query_filtered, keypoints_train_filtered, kp_query_filt_out, kp_train_filt_out;
    cv::Mat descriptors_query, descriptors_train;
    cv::Mat matches_image, filter_matches_image;

    auto Detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 1e-4f, 8);
    Detector->detectAndCompute(im1, cv::Mat(), keypoints_query, descriptors_query);
    Detector->detectAndCompute(im2, cv::Mat(), keypoints_train, descriptors_train);

    std::vector<cv::DMatch> matches = KpMatch(keypoints_query, descriptors_query, keypoints_train, descriptors_train, im1, im2, keypoints_query_filtered, keypoints_train_filtered);
    std::vector<cv::DMatch> goodMatches;

    std::vector<cv::Point2f> points_query, points_train; //rellenar

    for (size_t i = 0; i < keypoints_query_filtered.size(); i++)
    {
        points_query.push_back(keypoints_query_filtered[i].pt);
        points_train.push_back(keypoints_train_filtered[i].pt);
    }

    cv::drawMatches(im1, keypoints_query_filtered, im2, keypoints_train_filtered, matches, matches_image);

    cv::namedWindow("Matches", CV_WINDOW_NORMAL);
    cv::imshow("Matches", matches_image);

    cv::Mat inliers;

    F = cv::findFundamentalMat(points_query, points_train, cv::FM_RANSAC, 0.999, 1.0, 1000, inliers);

    for (size_t i = 0; i < inliers.total(); i++)
    {
        if (inliers.ptr<uchar>(0)[i])
        {
            int current_size = static_cast<int>(kp_query_filt_out.size());
            goodMatches.push_back(cv::DMatch(current_size, current_size, 0));
            kp_query_filt_out.push_back(keypoints_query_filtered[matches[i].queryIdx]);
            kp_train_filt_out.push_back(keypoints_train_filtered[matches[i].trainIdx]);
        }
    }

    cv::drawMatches(im1, kp_query_filt_out, im2, kp_train_filt_out, goodMatches, filter_matches_image);

    cv::namedWindow("Good Matches", CV_WINDOW_NORMAL);
    cv::imshow("Good Matches", filter_matches_image);

    return F;
}

void __recoverPose(cv::Mat E, std::vector<cv::Point2f> points_query, std::vector<cv::Point2f> points_train, cv::Mat camK, cv::Mat &R, cv::Mat &t)
{

    //normalize the 2d points
    cv::undistortPoints(points_query, points_query, camK, cv::Mat::zeros(1, 5, CV_64F));
    cv::undistortPoints(points_train, points_train, camK, cv::Mat::zeros(1, 5, CV_64F));
    //obtain R|t from the Essential Matrix
    cv::recoverPose(E, points_query, points_train, R, t);
}

//given two keypoints, the camera matrix K and the R|t transform from query to train
//obtains the corresponding 3d point
//returns true if the 3dpoint is correct and false otherwise
bool TriangulatePoint(const cv::KeyPoint &kp_query, const cv::KeyPoint &kp_train,
                      const cv::Mat &kin, const cv::Mat &rin, const cv::Mat &tin, cv::Point3f &p3d)
{
    auto getcosParallax = [](cv::Point3d p3d, const cv::Mat &R, const cv::Mat &t)
    {
        cv::Mat r64, t64;
        R.convertTo(r64, CV_64F);
        t.convertTo(t64, CV_64F);
        cv::Mat O1 = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat O2 = -R.t() * t;
        cv::Mat p3dC1(3, 1, CV_64F, &p3d);
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);
        return normal1.dot(normal2) / (dist1 * dist2);
    };
    auto project = [](const cv::Point3f p, cv::Mat &P)
    {
        cv::Point3f res;
        double *p34 = P.ptr<double>(0);
        res.x = p.x * p34[0] + p.y * p34[1] + p.z * p34[2] + p34[3];
        res.y = p.x * p34[4] + p.y * p34[5] + p.z * p34[6] + p34[7];
        res.z = p.x * p34[8] + p.y * p34[9] + p.z * p34[10] + p34[11];
        res.x /= res.z;
        res.y /= res.z;
        return cv::Point2f(res.x, res.y);
    };
    //make it 64f
    cv::Mat R, K, t;
    if (R.total() == 3)
        cv::Rodrigues(rin, R);
    else
        rin.convertTo(R, CV_64F);
    R.convertTo(R, CV_64F);
    kin.convertTo(K, CV_64F);
    tin.convertTo(t, CV_64F);
    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3, 4, CV_64F, cv::Scalar(0));
    K.copyTo(P1.rowRange(0, 3).colRange(0, 3));

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3, 4, CV_64F);
    R.copyTo(P2.rowRange(0, 3).colRange(0, 3));
    t.copyTo(P2.rowRange(0, 3).col(3));
    P2 = K * P2;

    cv::Mat A(4, 4, CV_64F);

    A.row(0) = kp_query.pt.x * P1.row(2) - P1.row(0);
    A.row(1) = kp_query.pt.y * P1.row(2) - P1.row(1);
    A.row(2) = kp_train.pt.x * P2.row(2) - P2.row(0);
    A.row(3) = kp_train.pt.y * P2.row(2) - P2.row(1);

    cv::Mat u, w, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat x3D = vt.row(3).t();
    x3D = x3D.rowRange(0, 3) / x3D.at<double>(3);

    p3d = cv::Point3f(x3D.at<double>(0), x3D.at<double>(1), x3D.at<double>(2));
    ;
    //check the point
    if (p3d.z < 0)
        return false; //behind the camera
    if (getcosParallax(p3d, R, t) < 0.99)
        return false; //too little parallax
    //check the reprojection error is low

    auto pp1 = project(p3d, P1);
    if (cv::norm(pp1 - kp_query.pt) > 1.5)
        return false;
    auto pp2 = project(p3d, P2);
    if (cv::norm(pp2 - kp_train.pt) > 1.5)
        return false;
    return true;
}

std::vector<cv::Point3f> Triangulate(cv::Mat im1, cv::Mat im2, cv::Mat F, CP cp)
{
    std::vector<cv::Point3f> _3dpoints;

    std::vector<cv::KeyPoint> keypoints_query, keypoints_train, keypoints_query_filtered, keypoints_train_filtered, kp_query_filt_out, kp_train_filt_out;
    std::vector<cv::Point2f> points_query, points_train;
    cv::Mat descriptors_query, descriptors_train;
    cv::Mat matches_image, filter_matches_image;

    auto Detector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 1e-4f, 8);
    Detector->detectAndCompute(im1, cv::Mat(), keypoints_query, descriptors_query);
    Detector->detectAndCompute(im2, cv::Mat(), keypoints_train, descriptors_train);

    // Obtenemos los matches esta vez pasando F para mejores resultados
    std::vector<cv::DMatch> matches = KpMatch(keypoints_query, descriptors_query, keypoints_train, descriptors_train, im1, im2, keypoints_query_filtered, keypoints_train_filtered, F);

    // Cálculo de la matriz esencial para poder obtener las matrices de rotación y translación en coordenadas reales
    cv::Mat E = (cp.camera_matrix.t() * F) * cp.camera_matrix;

    for (size_t i = 0; i < keypoints_query_filtered.size(); i++)
    {
        points_query.push_back(keypoints_query_filtered[i].pt); //627
        points_train.push_back(keypoints_train_filtered[i].pt);
    }

    // __recoverpose necesita que se le filtren los outliers a los puntos, por lo que volvemos a calcularlos
    cv::Mat inliers;

    F = cv::findFundamentalMat(points_query, points_train, cv::FM_RANSAC, 0.999, 1.0, 1000, inliers);

    std::vector<cv::Point2f> points_query_filt_out, points_train_filt_out;

    for (size_t i = 0; i < inliers.total(); i++)
    {
        if (inliers.ptr<uchar>(0)[i])
        {
            points_query_filt_out.push_back(keypoints_query_filtered[matches[i].queryIdx].pt);
            points_train_filt_out.push_back(keypoints_train_filtered[matches[i].trainIdx].pt);
        }
    }

    __recoverPose(E, points_query_filt_out, points_train_filt_out, cp.camera_matrix, cp.R, cp.t);

    std::cout << "R: " << cp.R << std::endl;
    std::cout << "t: " << cp.t << std::endl;

    //triangulate
    for (size_t i = 0; i < keypoints_query_filtered.size(); i++)
    {
        cv::Point3f p3d_temp;
        if (TriangulatePoint(keypoints_query_filtered[matches[i].queryIdx], keypoints_train_filtered[matches[i].trainIdx], cp.camera_matrix, cp.R, cp.t, p3d_temp))
        {
            _3dpoints.push_back(p3d_temp);
        }
    }

    return _3dpoints; //337
}
