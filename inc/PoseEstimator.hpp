#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/aruco.hpp>


class PoseEstimator
{
    cv::Matx33d mCameraMatrix;
    cv::Vec<double, 5> mDistCoeffs;
    cv::Ptr<cv::aruco::Dictionary> mDictionary;

public:
    PoseEstimator(cv::Matx33d cameraMatrix, cv::Vec<double, 5> distCoeffs, cv::aruco::PREDEFINED_DICTIONARY_NAME dictType = cv::aruco::DICT_6X6_250);
    void estimateMarkerPoses(std::vector<int> &ids, std::vector<std::vector<cv::Point2f>> &corners, cv::Mat &rVec, cv::Mat &tVec, const cv::Mat &image);
    cv::Mat drawArUcoMarkersAndAxisOnImage(const cv::Mat &image, const std::vector<int> &ids, const std::vector<std::vector<cv::Point2f>> &corners, const cv::Mat &rVec, const cv::Mat &tVec);
    cv::Matx44d computeCameraPoseMatrixFromBoardPose(const cv::Mat &rVec, const cv::Mat &tVec);
};