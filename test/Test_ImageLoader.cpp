#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"

#include "../inc/ImageLoader.hpp"

TEST(ImageLoaderTest, ReadSingleImage) {
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> masks;
    std::vector<Eigen::Matrix4d> transformations;
    std::vector<std::string> filenames = {"../resources/test.png"};
    std::vector<cv::Rect> rects = {cv::Rect(50, 50, 380, 380)};
    std::vector<cv::Mat> foregroundImages;
    
    ImageLoader::readImages(images, transformations, masks, filenames, rects);

    cv::imshow("ImageLoaderTest", images[0]);
    cv::waitKey(0);
    cv::destroyWindow("Test");
}

TEST(ImageLoaderTest, ReadSingleImageAndExtractForeground) {
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> masks;
    std::vector<Eigen::Matrix4d> transformations;
    std::vector<std::string> filenames = {"../resources/test.png"};
    std::vector<cv::Rect> rects = {cv::Rect(50, 50, 380, 380)};
    std::vector<cv::Mat> foregroundImages;
    
    ImageLoader::readImages(images, transformations, masks, filenames, rects);
    ImageLoader::extractForegroundImages(foregroundImages, images, masks);

    cv::imshow("Test Foreground Image", foregroundImages[0]);
    cv::waitKey(0);
    cv::destroyWindow("Test Foreground Image");
}
