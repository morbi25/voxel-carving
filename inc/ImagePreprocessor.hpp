#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

class ImagePreprocessor
{
public:
    static void extractForegroundImage(cv::Mat &foregroundImage, cv::Mat &image, const cv::Mat &mask);
    static void readImage(cv::Mat &image, cv::Mat &mask, const std::string &filename, const cv::Rect &rect, const unsigned int iterCount = 1, const cv::GrabCutModes &mode = cv::GC_INIT_WITH_RECT);
    static void readImagesAndComputeCameraPoses(std::vector<cv::Mat> &images, std::vector<cv::Mat> &transformations, std::vector<cv::Mat> &masks, const std::vector<std::string> &filenames, const std::vector<cv::Rect> &rects, const unsigned int iterCount = 1, const cv::GrabCutModes &mode = cv::GC_INIT_WITH_RECT);
};
