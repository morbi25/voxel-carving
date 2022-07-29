#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/aruco.hpp>

struct ImageMeta;

class PoseEstimator
{
    cv::Matx33d mCameraMatrix;
    cv::Vec<double, 5> mDistCoeffs;
    cv::Ptr<cv::aruco::Dictionary> mDictionary;

public:
    /**
     * @brief Construct a new Pose Estimator object accepting @p cameraMatrix, @p distCoeffs, @p dictType
     *
     * @param cameraMatrix Camera intrinsics matrix
     * @param distCoeffs Distortion coefficients
     * @param dictType Dictionary of markers employed for this board
     */
    PoseEstimator(cv::Matx33d cameraMatrix, cv::Vec<double, 5> distCoeffs, cv::aruco::PREDEFINED_DICTIONARY_NAME dictType = cv::aruco::DICT_6X6_250);

    /**
     * @brief Detect AruCo markers and estimate the pose of the ArUco board
     *
     * @param ids Output vector of identifiers for each marker in corners
     * @param corners Output vector of already detected markers corners; for each marker, its four corners are provided
     * @param rVec Output vector corresponding to the rotation vector of the board
     * @param tVec 	Output vector corresponding to the translation vector of the board
     * @param image Image for which the markers should be detected and the pose be estimated
     * @return True if at least one marker was detected and the pose could be estimated, otherwise false
     */
    bool detectMarkersAndEstimateBoardPose(std::vector<int> &ids, std::vector<std::vector<cv::Point2f>> &corners, cv::Mat &rVec, cv::Mat &tVec, ImageMeta imageMeta);

    /**
     * @brief Draw ArUCo markers and x-, y-, z-axis on the image
     *
     * @param image Image on which the markers and axes should be drawn
     * @param ids Vector of identifiers for each marker in corners
     * @param corners Vector of already detected markers corners
     * @param rVec Vector corresponding to the rotation vector of the board
     * @param tVec Vector corresponding to the translation vector of the board
     * @return Image on which the markers and axes are drawn
     */
    cv::Mat drawArUcoMarkersAndAxisOnImage(const cv::Mat &image, const std::vector<int> &ids, const std::vector<std::vector<cv::Point2f>> &corners, const cv::Mat &rVec, const cv::Mat &tVec);

    /**
     * @brief Compute the camera pose matrix given @p rVec and @p tVec determining the ArUco board pose
     *
     * @param rVec Vector corresponding to the rotation vector of the board
     * @param tVec Vector corresponding to the translation vector of the board
     * @return Rigid body transformation that transforms points from the board coordinate system to the camera coordinate system
     */
    cv::Matx44d computeCameraPoseMatrixFromBoardPose(const cv::Mat &rVec, const cv::Mat &tVec);
};