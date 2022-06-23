#include <opencv2/imgcodecs.hpp>

#include "../inc/ImageLoader.hpp"

Eigen::Matrix4d ImageLoader::computeCameraPoseFromArUcoMarkers(cv::Mat image)
{
    // ToDo:
    return Eigen::Matrix4d::Identity();
}

void ImageLoader::readImages(std::vector<cv::Mat> &images, std::vector<Eigen::Matrix4d> &transformations, std::vector<cv::Mat> &masks, const std::vector<std::string> &filenames, const std::vector<cv::Rect> &rects, const unsigned int iterCount, const cv::GrabCutModes &mode)
{
    // Temporary background and foreground model
    cv::Mat bgdModel;
    cv::Mat fgdModel;
    cv::Mat mask;
    //std::vector<Eigen::Matrix4d> cameraPoses;

    // Determine if masks should be used
    size_t maskSize = (mode == cv::GrabCutModes::GC_INIT_WITH_MASK ? masks.size() : filenames.size());

    // Iterate over all images in the sequence
    for (unsigned int i = 0; i < std::min({filenames.size(), rects.size(), maskSize}); i++)
    {
        // Read image
        auto image = cv::imread(filenames[i]);

        // Use mask if specified
        if (mode == cv::GrabCutModes::GC_INIT_WITH_MASK)
        {
            mask = masks[i];
        }

        // Compute rigid transformation
        transformations.push_back(computeCameraPoseFromArUcoMarkers(image));

        // Extract forground image
        cv::grabCut(image, mask, rects[i], bgdModel, fgdModel, iterCount, mode);

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

        // Add images and masks
        images.push_back(image);
        masks.push_back(mask);
    }
}

void ImageLoader::extractForegroundImages(std::vector<cv::Mat> &foregroundImages, std::vector<cv::Mat> &images, const std::vector<cv::Mat> &masks)
{
    // Iterate over all images and mask background out
    for (unsigned int i = 0; i < images.size() && i < masks.size(); i++)
    {
        cv::Mat foregroundImage;
        images[i].copyTo(foregroundImage, masks[i]);
        foregroundImages.push_back(foregroundImage);
    }
}
