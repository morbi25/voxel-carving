#include <opencv2/imgcodecs.hpp>

#include "../inc/ImagePreprocessor.hpp"

void ImagePreprocessor::extractForegroundImage(cv::Mat &foregroundImage, cv::Mat &image, const cv::Mat &mask)
{
    image.copyTo(foregroundImage, mask);
}

void ImagePreprocessor::readImage(cv::Mat &image, cv::Mat &mask, const std::string &filename, const cv::Rect &rect, const unsigned int iterCount, const cv::GrabCutModes &mode)
{
    // Temporary background and foreground model
    cv::Mat bgdModel;
    cv::Mat fgdModel;

    // Read image
    image = cv::imread(filename);

    // Use mask if specified
    if (mode == cv::GrabCutModes::GC_INIT_WITH_MASK)
    {
        mask = mask;
    }

    // Extract forground image
    cv::grabCut(image, mask, rect, bgdModel, fgdModel, iterCount, mode);

    // Convert possible background pixels also to background value
    cv::Mat foregroundImage(image);
    for (int j = 0; j < image.rows; j++)
    {
        for (int k = 0; k < image.cols; k++)
        {
            if (mask.at<uchar>(j, k) == cv::GrabCutClasses::GC_BGD || mask.at<uchar>(j, k) == cv::GrabCutClasses::GC_PR_BGD)
            {
                mask.at<uchar>(j, k) = cv::GrabCutClasses::GC_BGD;
            }
        }
    }
}

void ImagePreprocessor::readImagesAndComputeCameraPoses(std::vector<cv::Mat> &images, std::vector<cv::Mat> &transformations, std::vector<cv::Mat> &masks, const std::vector<std::string> &filenames, const std::vector<cv::Rect> &rects, const unsigned int iterCount, const cv::GrabCutModes &mode)
{
    // Temporary data
    cv::Mat image;
    cv::Mat mask;

    // Determine if masks should be used
    size_t maskSize = (mode == cv::GrabCutModes::GC_INIT_WITH_MASK ? masks.size() : filenames.size());

    // Iterate over all images in the sequence
    for (unsigned int i = 0; i < std::min({filenames.size(), rects.size(), maskSize}); i++)
    {

        readImage(image, mask, filenames[i], rects[i], iterCount, mode);

        // ToDo: Compute rigid transformation
        // transformations.push_back(computeCameraPoseFromArUcoMarkers(image));

        images.push_back(image);
        masks.push_back(mask);
    }
}
