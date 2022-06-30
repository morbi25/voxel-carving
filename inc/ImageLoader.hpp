#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "Eigen.h"

class ImageLoader
{
    static Eigen::Matrix4d computeCameraPoseFromArUcoMarkers(cv::Mat image);

public:
    static void readImages(std::vector<cv::Mat> &images, std::vector<Eigen::Matrix4d> &transformations, std::vector<cv::Mat> &mask, const std::vector<std::string> &filenames, const std::vector<cv::Rect> &rects, const unsigned int iterCount = 1, const cv::GrabCutModes &mode = cv::GC_INIT_WITH_RECT);
    static void extractForegroundImages(std::vector<cv::Mat>& foregroundImages, std::vector<cv::Mat> &images, const std::vector<cv::Mat> &mask);
};
