#pragma once

#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "PoseEstimator.hpp"
#include "ForegroundSegmenter.hpp"

// Struct that captures an image, its corresponding foreground image, its camera pose, and its file path
struct ImageMeta
{
    cv::Mat image;
    cv::Mat foregroundImage;
    cv::Matx44d cameraPose;
    std::filesystem::path filepath;
};

/// Class that pre-processes an input image dataset for voxel-carving
class ImagePreprocessor
{
    PoseEstimator &mPoseEstimator;
    ForegroundSegmenter &mForegroundSegmenter;
    bool mDoForegroundSegmentation;
    double mScale;

public:
    /**
     * @brief Construct a new ImagePreprocessor object accepting @p poseEstimator, @p foregroundSegmenter, @p doForegroundSegmentation, and @p scale
     *
     * @param poseEstimator PoseEstimator instance used to estimate the camera pose in world coordinates for each image from the dataset
     * @param foregroundSegmenter Concrete ForgroundSegmenter instance (GrabCut, ColorThreshold, Unet)
     * @param doForegroundSegmentation Indicates whether to perform foreground segmentation
     * @param scale Scale factor to resize the input images
     */
    ImagePreprocessor(PoseEstimator &poseEstimator, ForegroundSegmenter &foregroundSegmenter, bool doForegroundSegmentation = true, double scale = 0.125);

    /**
     * @brief Read a dataset of images being all in one specified directory and compute the camera poses for each
     *
     * @param inDir Directory for the dataset that should be processed
     * @param drawMarkersAndAxesOnImage Flag that specifies whether the detected markers and axes should be drawn on the images
     * @return Vector storing all images and its meta data
     */
    std::vector<ImageMeta> readImagesAndComputeCameraPoses(const std::string &inDir, bool drawMarkersAndAxisOnImage = false);

    /**
     * @brief Stores and prints out verbose information (for debugging)
     *
     * @param imageMetas Images and its corresponding meta data
     * @param outDir Output directory where the pre-processed images should be stored
     * @param writeImage Flag indicating whether to store the pre-processed input image
     * @param writeForegroundImage Flag indicating whether to store the computed foreground image
     * @param printCameraPoses Flag indicating whether to print the calculated camera poses
     */
    void verbose(std::vector<ImageMeta> imageMetas, std::string outDir, bool writeImage, bool writeForegroundImage, bool printCameraPoses);
};
