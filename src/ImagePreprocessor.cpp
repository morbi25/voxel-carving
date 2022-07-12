#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>

#include "../inc/ImagePreprocessor.hpp"

ImagePreprocessor::ImagePreprocessor(PoseEstimator poseEstimator) : mPoseEstimator(poseEstimator)
{
}

ImagePreprocessor::ImagePreprocessor(cv::Matx33d cameraMatrix, cv::Vec<double, 5> distCoeffs, cv::aruco::PREDEFINED_DICTIONARY_NAME dictType)
    : mPoseEstimator(cameraMatrix, distCoeffs)
{
}

void ImagePreprocessor::doForegroundSegmentation(cv::Mat &foregroundImage, cv::Mat &mask, const cv::Mat &image, const cv::Rect &rect, const unsigned int iterCount, const cv::GrabCutModes &mode, const bool &objectIsBlack)
{
    // Temporary background and foreground model
    cv::Mat bgdModel;
    cv::Mat fgdModel;

    // Use mask if specified
    if (mode == cv::GrabCutModes::GC_INIT_WITH_MASK)
    {
        mask = mask;
    }

    // Extract forground mask
    cv::grabCut(image, mask, rect, bgdModel, fgdModel, iterCount, mode);

    // Convert possible background pixels also to background value
    for (int j = 0; j < image.rows; j++)
    {
        for (int k = 0; k < image.cols; k++)
        {
            if (mask.at<uchar>(j, k) == cv::GrabCutClasses::GC_BGD || mask.at<uchar>(j, k) == cv::GrabCutClasses::GC_PR_BGD)
            {
                mask.at<uchar>(j, k) = cv::GrabCutClasses::GC_BGD;
            }
            cv::Vec3b pixel = image.at<cv::Vec3b>(j, k);
            if (mask.at<uchar>(j, k) == cv::GrabCutClasses::GC_FGD || mask.at<uchar>(j, k) == cv::GrabCutClasses::GC_PR_FGD)
            {
                double BRTreshold = pixel.val[1] * 1.3;
                if (pixel.val[1] < 40 || (double)pixel.val[0] + (double)pixel.val[2] > BRTreshold)
                {
                    mask.at<uchar>(j, k) = cv::GrabCutClasses::GC_BGD;
                }
            }
        }
    }

    // Copy only foreground to foreground image and mask out background region
    // If the object of interest is black, we set the background color to white, otherwise to black
    if (objectIsBlack)
    {
        cv::Mat maskTemp(mask.size(), mask.type());
        maskTemp.setTo(cv::Scalar(255), mask);
        cv::Mat invMask(mask.size(), mask.type());
        cv::bitwise_not(maskTemp, invMask);
        image.copyTo(foregroundImage, mask);
        foregroundImage.setTo(cv::Scalar(255, 255, 255), invMask);
    }
    else
    {
        image.copyTo(foregroundImage, mask);
    }
}

void ImagePreprocessor::readImagesAndComputeCameraPoses(std::vector<std::filesystem::path> &filepaths, std::vector<cv::Mat> &images, std::vector<cv::Matx44d> &cameraPoses, const std::string &inDir, const bool &drawMarkersAndAxisOnImage
#ifdef DO_FOREGROUND_SEGMENTATION
                                                        ,
                                                        std::unordered_map<std::string, cv::Rect> &imageNamesToRect, const unsigned int &iterCount, const cv::GrabCutModes &mode, const std::vector<cv::Mat> &masks
#endif
)
{
    // Temporary data
    cv::Mat image;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat rVec;
    cv::Mat tVec;
    cv::Mat cameraPose;

    auto dirIterator = std::filesystem::recursive_directory_iterator(inDir);
    unsigned int i = 0;

#ifdef DO_FOREGROUND_SEGMENTATION
    cv::Mat foregroundImage;
    cv::Mat mask;
    for (auto &rect : imageNamesToRect)
    {
        rect.second.x *= 0.125;
        rect.second.y *= 0.125;
        rect.second.width *= 0.125;
        rect.second.height *= 0.125;
    }
#endif

    // Iterate over all images in the sequence
    for (const auto &dirEntry : std::filesystem::recursive_directory_iterator(inDir))
    {
#ifdef DO_FOREGROUND_SEGMENTATION
        // Check if masks and rects are valid
        if (i >= imageNamesToRect.size() || mode == cv::GC_INIT_WITH_MASK && i >= masks.size())
        {
            break;
        }
#endif
        // Get filesystem path
        auto filepath = dirEntry.path();

        // Read image
        image = cv::imread(filepath.string());

        // Estimate marker poses
        mPoseEstimator.estimateMarkerPoses(ids, corners, rVec, tVec, image);

        // Draw ArUco markers and axis on image
        if (drawMarkersAndAxisOnImage)
        {
            image = mPoseEstimator.drawArUcoMarkersAndAxisOnImage(image, ids, corners, rVec, tVec);
        }

        // Compute camera pose matrix with respect to board
        cv::Matx44d worldToCamera = mPoseEstimator.computeCameraPoseMatrixFromBoardPose(rVec, tVec);

#ifdef RESIZE_IMAGES
        // Resize images to have a resolution that can be handled
        cv::resize(image, image, cv::Size(), 0.125, 0.125);
#endif
#ifdef DO_FOREGROUND_SEGMENTATION
        // Compute foreground mask and foreground image
        ImagePreprocessor::doForegroundSegmentation(foregroundImage, mask, image, imageNamesToRect[filepath.filename().string()], iterCount, mode);
#endif

        // Store all relevant data
        filepaths.push_back(filepath);
#ifdef DO_FOREGROUND_SEGMENTATION
        images.push_back(foregroundImage);
#else
        images.push_back(image);
#endif
        cameraPoses.push_back(worldToCamera);

        // Clear all temporary data structures and increment i
        // image.release();
#ifdef DO_FOREGROUND_SEGMENTATION
        foregroundImage.release();
        mask.release();
#endif
        ids.clear();
        corners.clear();
        rVec.release();
        tVec.release();
        i++;
    }
}
