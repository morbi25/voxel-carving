#include <iostream>
#include <opencv2/calib3d.hpp>

#include "../inc/PoseEstimator.hpp"

PoseEstimator::PoseEstimator(cv::Matx33d cameraMatrix, cv::Vec<double, 5> distCoeffs, cv::aruco::PREDEFINED_DICTIONARY_NAME dictType)
    : mCameraMatrix(std::move(cameraMatrix)), mDistCoeffs(std::move(distCoeffs)), mDictionary(cv::aruco::getPredefinedDictionary(dictType))
{
}

bool PoseEstimator::estimateMarkerPoses(std::vector<int> &ids, std::vector<std::vector<cv::Point2f>> &corners, cv::Mat &rVec, cv::Mat &tVec, const cv::Mat &image)
{
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(5, 7, 0.04, 0.01, mDictionary);

    cv::aruco::detectMarkers(image, board->dictionary, corners, ids);

    // Check if one marker is detected
    if (ids.size() > 0)
    {
        cv::aruco::estimatePoseBoard(corners, ids, board, mCameraMatrix, mDistCoeffs, rVec, tVec);
        return true;
    }
    else
    {
        std::cout << ("Cannot determine camera pose since no ArUco marker was detected.") << std::endl;
        return false;
    }
}

cv::Mat PoseEstimator::drawArUcoMarkersAndAxisOnImage(const cv::Mat &image, const std::vector<int> &ids, const std::vector<std::vector<cv::Point2f>> &corners, const cv::Mat &rVec, const cv::Mat &tVec)
{
    // Create copy of image to store also markers
    cv::Mat imageWithMarkers;
    image.copyTo(imageWithMarkers);

    cv::aruco::drawDetectedMarkers(imageWithMarkers, corners, ids);

    // Draw axis for each marker
    cv::drawFrameAxes(imageWithMarkers, mCameraMatrix, mDistCoeffs, rVec, tVec, 0.1);

    return imageWithMarkers;
}

cv::Matx44d PoseEstimator::computeCameraPoseMatrixFromBoardPose(const cv::Mat &rVec, const cv::Mat &tVec)
{
    // Compute rotation matrix from vector
    cv::Mat cameraRMat;
    cv::Rodrigues(rVec, cameraRMat);

    // Bundle rotation and translation into one rigid transformation matrix
    cv::Mat cameraToWorld(cv::Mat::eye(4, 4, CV_64F));
    cv::Mat minor = cameraToWorld.colRange(0, 3).rowRange(0, 3);
    cameraRMat.copyTo(minor);
    minor = cameraToWorld.colRange(3, 4).rowRange(0, 3);
    tVec.copyTo(minor);

    cv::Matx44d cameraToWorld44;
    cameraToWorld.copyTo(cameraToWorld44);

    // According to docu
    // rvec Output vector [x,y,z] corresponding to the rotation vector of the board
    // tvec Output vector [x,y,z] corresponding to the translation vector of the board
    // To get the transformation from board to camera we must compute the inverse
    return cameraToWorld44;
}