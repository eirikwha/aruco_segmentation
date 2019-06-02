//
// Created by eirik on 09.05.19.
//

/*
 * Copyright (c) 2018 Flight Dynamics and Control Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

void drawCubeWireframe(
        cv::InputOutputArray image, cv::InputArray cameraMatrix,
        cv::InputArray distCoeffs, cv::InputArray rvec, cv::InputArray tvec,
        float l)
{

    CV_Assert(
            image.getMat().total() != 0 &&
            (image.getMat().channels() == 1 || image.getMat().channels() == 3)
    );
    CV_Assert(l > 0);
    float half_l = l / 2.0f;

    // project cube points
    std::vector<cv::Point3f> axisPoints;
    axisPoints.emplace_back(cv::Point3f(half_l, half_l, l));
    axisPoints.emplace_back(cv::Point3f(half_l, -half_l, l));
    axisPoints.emplace_back(cv::Point3f(-half_l, -half_l, l));
    axisPoints.emplace_back(cv::Point3f(-half_l, half_l, l));
    axisPoints.emplace_back(cv::Point3f(half_l, half_l, 0));
    axisPoints.emplace_back(cv::Point3f(half_l, -half_l, 0));
    axisPoints.emplace_back(cv::Point3f(-half_l, -half_l, 0));
    axisPoints.emplace_back(cv::Point3f(-half_l, half_l, 0));

    std::vector<cv::Point2f> imagePoints;
    projectPoints(
            axisPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints
    );

    // draw cube edges lines
    cv::line(image, imagePoints[0], imagePoints[1], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[0], imagePoints[4], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[1], imagePoints[2], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[1], imagePoints[5], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[2], imagePoints[3], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[2], imagePoints[6], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[3], imagePoints[7], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[4], imagePoints[5], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[4], imagePoints[7], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[5], imagePoints[6], cv::Scalar(255, 0, 0), 3);
    cv::line(image, imagePoints[6], imagePoints[7], cv::Scalar(255, 0, 0), 3);
}

void drawPlane(){} // TODO: Visualize plane

void drawAxis(cv::Mat image,std::vector<int> ids,
        cv::Mat camera_matrix, cv::Mat dist_coeffs, cv::Vec3d rvec, cv::Vec3d tvec, int pointNo) {

    std::ostringstream vector_to_marker;

    cv::aruco::drawAxis(image, camera_matrix, dist_coeffs,
                        rvec, tvec, 0.1);

    vector_to_marker.str(std::string());
    vector_to_marker << std::setprecision(4)
                     << "x" + std::to_string(pointNo) + ": " << std::setw(8) << tvec(0);
    cv::putText(image, vector_to_marker.str(),
                cvPoint(10, 30 + pointNo*70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cvScalar(0, 252, 124), 1, CV_AA);

    vector_to_marker.str(std::string());
    vector_to_marker << std::setprecision(4)
                     << "y" + std::to_string(pointNo) + ": " << std::setw(8) << tvec(1);
    cv::putText(image, vector_to_marker.str(),
                cvPoint(10, 50 + pointNo*70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cvScalar(0, 252, 124), 1, CV_AA);

    vector_to_marker.str(std::string());
    vector_to_marker << std::setprecision(4)
                     << "z" + std::to_string(pointNo) + ": " << std::setw(8) << tvec(2);
    cv::putText(image, vector_to_marker.str(),
                cvPoint(10, 70 + pointNo*70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cvScalar(0, 252, 124), 1, CV_AA);
}

static bool readDetectorParameters(std::string filename, cv::Ptr<cv::aruco::DetectorParameters> &params) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

std::vector<double> compute3PtPlaneEq(std::vector<cv::Vec3d> &tvecs)
{
    double a1 = tvecs[1][0] - tvecs[0][0];
    double b1 = tvecs[1][1] - tvecs[0][1];
    double c1 = tvecs[1][2] - tvecs[0][2];

    double a2 = tvecs[2][0] - tvecs[0][0];
    double b2 = tvecs[2][1] - tvecs[0][1];
    double c2 = tvecs[2][2] - tvecs[0][2];

    double a = b1 * c2 - b2 * c1;
    double b = a2 * c1 - a1 * c2;
    double c = a1 * b2 - b1 * a2;
    double d = (- a * tvecs[0][0] - b * tvecs[0][1] - c * tvecs[0][2]);

    std::cout << std::fixed;
    std::cout << "Plane equation: " << a << " x + " << b
              << " y + " << c << " z + " << d << " = 0." << std::endl;

    return std::vector<double> {a,b,c,d};
}

void pointToPlane(const std::vector<double> &planeEq, const cv::Vec3d &planePoint, std::vector<cv::Vec3d> points3d){

    double n_len = sqrt(pow(planeEq[0],2) + pow(planeEq[1],2)+ pow(planeEq[2],2));
    const std::vector<double> n_unit = {planeEq[0]/n_len, planeEq[1]/n_len, planeEq[2]/n_len};

    for (int i = 0; i <3; i++) {
        std::cout << n_unit[i] << " , " << std::endl;
    }

    double dist;
    for (int i =0; i<3;i++){
        dist += (points3d[0][i]-planePoint[i])*(n_unit[i]);
    }
    std::cout << "dist: " << dist << std::endl;
}


int main(int argc, char **argv)
{
    float actual_marker_length = atof(argv[1]);  // this should be in meters

    cv::Mat image, image_copy;
    cv::Mat camera_matrix, dist_coeffs;

    // TODO pass filepaths in argument, or use package//: (ros feature)

    image = cv::imread("/home/eirik/catkin_ws/src/aruco_segmentation/data/data3.png");
    cv::Ptr<cv::aruco::Dictionary> dictionary =
            cv::aruco::getPredefinedDictionary(10);

    cv::FileStorage fs("/home/eirik/catkin_ws/src/aruco_segmentation/data/intrinsics.yml", cv::FileStorage::READ);
    fs["cameraMatrix"] >> camera_matrix;
    fs["distCoeffs"] >> dist_coeffs;

    cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
    bool readOk = readDetectorParameters("/home/eirik/catkin_ws/src/aruco_segmentation/data/detector_params.yml", detectorParams);
    if(!readOk) {
        std::cerr << "Invalid detector parameters file" << std::endl;
        return 0;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids, detectorParams);

    // if at least one marker detected
    if (ids.size() > 0) {
        image_copy = image;
        cv::aruco::drawDetectedMarkers(image_copy, corners, ids);

        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_length,
                                             camera_matrix, dist_coeffs, rvecs, tvecs);

        // draw axis for each marker
        // draw cube on marker
        for (int i = 0; i < ids.size(); i++) {
            drawCubeWireframe(image_copy, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], actual_marker_length);
            drawAxis(image_copy, ids, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], i);
        }

        cv::imshow("Pose estimation", image_copy);
        cv::waitKey(0);

        for (int i = 0; i <3; i++) {
            std::cout << tvecs[i] << std::endl << std::endl;
        }

        std::vector<double> plane = compute3PtPlaneEq(tvecs);

        pointToPlane(plane,tvecs[0], tvecs);
    }

    else{
        std::cerr << "No markers detected" << std::endl;

    }

    //std::cout << std::endl << corners.at(0) << std::endl;
}
