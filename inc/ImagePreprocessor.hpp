#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "PoseEstimator.hpp"

// Configurations
#define DO_FOREGROUND_SEGMENTATION
#define RESIZE_IMAGES

class ImagePreprocessor
{
    PoseEstimator mPoseEstimator;
public:
    ImagePreprocessor(PoseEstimator poseEstimator);
    ImagePreprocessor(cv::Matx33d cameraMatrix, cv::Vec<double, 5> distCoeffs, cv::aruco::PREDEFINED_DICTIONARY_NAME dictType = cv::aruco::DICT_6X6_250);
    static void extractForegroundImage(cv::Mat &foregroundImage, cv::Mat &image, const cv::Mat &mask);
    static void doForegroundSegmentation(cv::Mat &foregroundImage, cv::Mat &mask, const cv::Mat &image, const cv::Rect &rect, const unsigned int iterCount = 1, const cv::GrabCutModes &mode = cv::GC_INIT_WITH_RECT, const bool &objectIsBlack = false);
    void readImagesAndComputeCameraPoses(std::vector<std::filesystem::path>& filepaths, std::vector<cv::Mat> &images, std::vector<cv::Matx44d> &cameraPoses, const std::string &inDir, const bool &drawMarkersAndAxisOnImage
    #ifdef DO_FOREGROUND_SEGMENTATION
    , std::unordered_map<std::string, cv::Rect> &imageNamesToRect, const unsigned int &iterCount = 1, const cv::GrabCutModes &mode = cv::GC_INIT_WITH_RECT, const std::vector<cv::Mat> &masks = {}
    #endif
    );
};
