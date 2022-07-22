#include "../inc/ForegroundSegmenter.hpp"

GrabCut::GrabCut(cv::Rect rect, unsigned int iterCount, cv::GrabCutModes mode, cv::Mat mask, double scale) : mRect(std::move(rect)), mIterCount(std::move(iterCount)), mMode(std::move(mode)), mMask(std::move(mask)), mScale(scale)
{
}

cv::Mat GrabCut::doForegroundSegmentation(const cv::Mat &image)
{
    // Temporary background and foreground model
    cv::Mat bgdModel;
    cv::Mat fgdModel;

    // Scale rectangle that exclude background area according to image resizing
    mRect.x *= mScale;
    mRect.y *= mScale;
    mRect.width *= mScale;
    mRect.height *= mScale;

    // Extract forground mask
    cv::grabCut(image, mMask, mRect, bgdModel, fgdModel, mIterCount, mMode);

    // Convert possible background pixels also to background values
    for (int j = 0; j < image.rows; j++)
    {
        for (int k = 0; k < image.cols; k++)
        {
            if (mMask.at<uchar>(j, k) == cv::GrabCutClasses::GC_BGD || mMask.at<uchar>(j, k) == cv::GrabCutClasses::GC_PR_BGD)
            {
                mMask.at<uchar>(j, k) = cv::GrabCutClasses::GC_BGD;
            }
        }
    }

    // Masking of the background to get the foreground image
    cv::Mat foregroundImage;
    image.copyTo(foregroundImage, mMask);
    return foregroundImage;
}

cv::Mat ColorThreshold::doForegroundSegmentation(const cv::Mat &image)
{
    cv::Mat foregroundImage = image;

    // Iterate over image pixels
    for (int j = 0; j < image.rows; j++)
    {
        for (int k = 0; k < image.cols; k++)
        {
            // Perform color thresholding
            cv::Vec3b pixel = image.at<cv::Vec3b>(j, k);
            double BRTreshold = pixel.val[1] * 1.3;
            if (pixel.val[1] < 40 || (double)pixel.val[0] + (double)pixel.val[2] > BRTreshold)
            {
                foregroundImage.at<cv::Vec3b>(j, k) = cv::Vec3b(0, 0, 0);
            }
        }
    }
    return image;
}