#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"

#include "../inc/ImagePreprocessor.hpp"

TEST(ImagePreprocessorTest, ReadSingleImage) {
    cv::Mat image;
    cv::Mat mask;
    std::string filename = "../resources/test1.png";
    cv::Rect rect = cv::Rect(50, 50, 380, 380);
    cv::Mat foregroundImage;
    
    ImagePreprocessor::readImage(image, mask, filename, rect);

    cv::imshow("Test Image", image);
    cv::waitKey(0);
    cv::destroyWindow("Test Image");
}

TEST(ImagePreprocessorTest, ReadSingleImageAndExtractForeground) {
    cv::Mat image;
    cv::Mat mask;
    std::string filename = "../resources/test1.png";
    cv::Rect rect = cv::Rect(50, 50, 400, 500);
    cv::Mat foregroundImage;
    
    ImagePreprocessor::readImage(image, mask, filename, rect);
    ImagePreprocessor::extractForegroundImage(foregroundImage, image, mask);

    cv::imwrite("../resources/test_foreground.png", foregroundImage);
    cv::imshow("Test Foreground Image", foregroundImage);
    cv::waitKey(0);
    cv::destroyWindow("Test Foreground Image");
}

TEST(ImagePreprocessorTest, ReadMultipleImages) {
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> masks;
    std::vector<cv::Mat> transformations;
    std::vector<std::string> filenames = {"../resources/test1.png", "../resources/test2.png"};
    std::vector<cv::Rect> rects = {cv::Rect(50, 50, 380, 380), cv::Rect(211, 137, 322, 376)};
    cv::Mat foregroundImage;
    
    ImagePreprocessor::readImagesAndComputeCameraPoses(images, transformations, masks, filenames, rects, 5);

    for (unsigned int i = 0; i < std::min(images.size(), masks.size()); i++)
    {
        ImagePreprocessor::extractForegroundImage(foregroundImage, images[i], masks[i]);
        cv::imshow("Test Image " + std::to_string(i), images[i]);
        cv::imwrite("../resources/test_foreground" + std::to_string(i) + ".png", foregroundImage);
        cv::imshow("Test Foreground Image " + std::to_string(i), foregroundImage);
        cv::waitKey(0);
        cv::destroyWindow("Test Image");
    }
}
