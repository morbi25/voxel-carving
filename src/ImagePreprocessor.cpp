#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <omp.h>

#include "../inc/ImagePreprocessor.hpp"

ImagePreprocessor::ImagePreprocessor(PoseEstimator &poseEstimator, ForegroundSegmenter &foregroundSegmenter, bool doForegroundSegmentation, double scale) : mPoseEstimator(poseEstimator), mForegroundSegmenter(foregroundSegmenter), mDoForegroundSegmentation(std::move(doForegroundSegmentation)), mScale(std::move(scale))
{
}

std::vector<ImageMeta> ImagePreprocessor::readImagesAndComputeCameraPoses(const std::string &inDir, bool drawMarkersAndAxisOnImage)
{
    // Return data
    std::vector<ImageMeta> imageMetas;
    ImageMeta imageMeta;

    // Temporary data
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat rVec;
    cv::Mat tVec;

    // Iterator to iterate over dataset directory
    auto dirIterator = std::filesystem::recursive_directory_iterator(inDir);

    // Iterate over all images in the sequence
    for (auto &entry : dirIterator)
    {
        // Get filesystem path
        imageMeta.filepath = entry.path();

        // Read image
        imageMeta.image = cv::imread(imageMeta.filepath.string());

        // Estimate marker poses
        if (!mPoseEstimator.detectMarkersAndEstimateBoardPose(ids, corners, rVec, tVec, imageMeta))
        {
            continue;
        }

        // Draw ArUco markers and axis on image
        if (drawMarkersAndAxisOnImage)
        {
            imageMeta.image = mPoseEstimator.drawArUcoMarkersAndAxisOnImage(imageMeta.image, ids, corners, rVec, tVec);
        }

        // Compute camera pose matrix with respect to board
        imageMeta.cameraPose = mPoseEstimator.computeCameraPoseMatrixFromBoardPose(rVec, tVec);

        // Resize images to have a resolution that can be handled
        if (mScale != 1.0)
        {
            cv::resize(imageMeta.image, imageMeta.image, cv::Size(), mScale, mScale);
        }

        // Peform foreground segmentation on input image
        cv::Mat foregroundImageBGR = mForegroundSegmenter.doForegroundSegmentation(imageMeta.image);
        imageMeta.foregroundImage = foregroundImageBGR;

        // Store all relevant data
        imageMetas.push_back(imageMeta);

        // Clear all temporary data structures and increment i
        foregroundImageBGR.release();
        ids.clear();
        corners.clear();
        rVec.release();
        tVec.release();
    }

    return imageMetas;
}

void ImagePreprocessor::verbose(std::vector<ImageMeta> imageMetas, std::string outDir, bool writeImage, bool writeForegroundImage, bool printCameraPoses)
{
    for (auto &imageMeta : imageMetas)
    {
        std::string filename = imageMeta.filepath.filename().string();
        if (writeImage)
        {
            cv::imwrite(outDir + filename.substr(0, filename.find_last_of(".")) + "_preprocessed.jpg", imageMeta.image);
        }
        if (writeForegroundImage)
        {
            cv::imwrite(outDir + filename.substr(0, filename.find_last_of(".")) + "_foreground.jpg", imageMeta.foregroundImage);
        }
        if (printCameraPoses)
        {
            std::cout << imageMeta.cameraPose << std::endl;
        }
    }
}
