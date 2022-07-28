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

cv::Mat U2Net::doForegroundSegmentation(const cv::Mat& image)
{
    int background_threshold = 10;
    int foreground_trehshold = 240;
    cv::Mat small_image;
    double min, max;
    cv::Mat foregroundImage = image;
    // Load network
    auto network = cv::dnn::readNetFromONNX("../resources/u2net.onnx");
    // Set Backend and perform prediction on CPU
    network.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    network.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    
    // All predictions are performed on a image of size 320 x 320, the mask will later be upsampled
    cv::resize(image, small_image, cv::Size(320, 320), cv::InterpolationFlags::INTER_LANCZOS4);
    
    // U2net expect float values as input
    small_image.convertTo(small_image, CV_32F);
    
    // Normalization of values acc to U2Net training spec
    cv::minMaxLoc(small_image, &min, &max);
    for (int j = 0; j < small_image.rows; j++)
    {
        for (int k = 0; k < small_image.cols; k++)
        {
            cv::Vec3f &pixel = small_image.at<cv::Vec3f>(j, k);
            pixel.val[0] = (pixel.val[0]  / max - 0.485) / 0.229;
            pixel.val[1] = (pixel.val[1] / max - 0.456) / 0.224;
            pixel.val[2] = (pixel.val[2] / max - 0.406) / 0.225;
          
        }
    }

    std::vector< std::vector< cv::Mat > > outBlobs;
    // Get all outputs of the u2net, the names are read out of the onnx file. Netron can be used for this.
    std::vector< cv::String > outBlobNames = { "1959", "1960","1961" ,"1962" ,"1963" ,"1964" ,"1965" };
    
    // Define the downsampled image as input 
    network.setInput(cv::dnn::blobFromImage(small_image));

    
    // Perform inference
    network.forward(outBlobs, outBlobNames);
    
    // Combine the 7 output images into one grayscale image
    for (int i = 0; i < 6; i++)
    {
        int x = (outBlobs[i][0]).size[2];
        int y = (outBlobs[i][0]).size[2];

        cv::Size size(x, y);
        outBlobs[i][0] = cv::Mat(size, CV_32F, (outBlobs[i][0]).ptr(0, 0));
        resize(outBlobs[i][0], outBlobs[i][0], image.size(), cv::InterpolationFlags::INTER_LANCZOS4);
    }
    
    cv::Mat combinedOutput = (cv::Mat)outBlobs[0][0];

    for (int i = 0 + 1; i < 6; i++)
    {
        cv::add(combinedOutput, outBlobs[i][0], combinedOutput);
    }
    cv::normalize(combinedOutput, combinedOutput, 0, 255, cv::NORM_MINMAX);
    


    // Generate trimap from the grayscale output
    cv::Mat trimap = combinedOutput;

    for (int j = 0; j < trimap.rows; j++)
    {
        for (int k = 0; k < trimap.cols; k++)
        {
            if (trimap.at<float>(j, k) >foreground_trehshold)
            {
                trimap.at<float>(j, k) = 255;
            }
            else if (trimap.at<float>(j, k) < background_threshold)
            {
                trimap.at<float>(j, k) = 0;
            }
            else
            {
                trimap.at<float>(j, k) = 128;
            }



        }
    }
    trimap.convertTo(trimap, CV_8U);

    cv::Mat alphamat_out;
    cv::Mat output_Image = image;

    // Use alphamat together with the trimap to generate mask
    cv::alphamat::infoFlow(image, trimap, alphamat_out);
    
    // Apply mask
    for (int j = 0; j < alphamat_out.rows; j++)
    {
        for (int k = 0; k < alphamat_out.cols; k++)
        {
            int value = alphamat_out.at<uchar>(j, k);
            if (value < 210)
            {
                output_Image.at<cv::Vec3b>(j, k) = cv::Vec3b(0, 0, 0);
            }


        }
    }
       
    return output_Image;
}

