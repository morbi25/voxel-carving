#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

/// Class of an abstract ForegroundSegmenter
class ForegroundSegmenter
{
public:
    /**
     * @brief Perform foreground segmentation
     *
     * @param image Image for which the foreground should be extracted
     * @return Foreground image
     */
    virtual cv::Mat doForegroundSegmentation(const cv::Mat &image) = 0;
};

/// Class of a ForegroundSegmenter that uses grabcut
class GrabCut : public ForegroundSegmenter
{
    cv::Rect mRect;
    unsigned int mIterCount;
    cv::GrabCutModes mMode;
    cv::Mat mMask;
    double mScale;

public:
    /**
     * @brief Construct a new Grab Cut object @p rect, @p iterCount, @p mode, @p mask, and @p scale
     *
     * @param rect ROI containing a segmented object, the pixels outside of the ROI are marked as "obvious background"
     * @param iterCount Number of iterations the algorithm should make before returning the result (the result can be refined with further calls with mode==GC_INIT_WITH_MASK or mode==GC_EVAL)
     * @param mode Operation mode that could be one of the GrabCutModes
     * @param mask 8-bit single-channel mask (the mask is initialized by the function when mode is set to GC_INIT_WITH_RECT)
     * @param scale Scale factor for scaling the rectangle
     */
    GrabCut(cv::Rect rect, unsigned int iterCount = 1, cv::GrabCutModes mode = cv::GC_INIT_WITH_RECT, cv::Mat mask = cv::Mat(), double scale = 0.125);

    /**
     * @brief Perform foreground segmentation using grabcut
     *
     * @param image Image for which the foreground should be extracted
     * @return Foreground image
     */
    cv::Mat doForegroundSegmentation(const cv::Mat &image) override;
};

/// Class of a ForegroundSegmenter that uses color thresholding
class ColorThreshold : public ForegroundSegmenter
{
public:
    /**
     * @brief Perform foreground segmentation using color thresholding
     *
     * @param image Image for which the foreground should be extracted
     * @return Foreground image
     */
    cv::Mat doForegroundSegmentation(const cv::Mat &image) override;
};

// ToDo: Create an additional class to use Unet for foreground segmentation