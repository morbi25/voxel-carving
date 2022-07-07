#include <filesystem>
#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"

#include "../inc/ImagePreprocessor.hpp"

TEST(ImagePreprocessorTest, ReadSingleImageAndExtractForeground)
{
    cv::Mat image;
    cv::Mat mask;
    std::string filename = "../resources/test0.png";
    cv::Rect rect = cv::Rect(50, 50, 400, 500);
    cv::Mat foregroundImage;

    image = cv::imread(filename);
    ImagePreprocessor::doForegroundSegmentation(foregroundImage, mask, image, rect);

    cv::imwrite("../resources/test_foreground.png", foregroundImage);
}

TEST(ImagePreprocessorTest, ReadTeapotImageAndExtractForeground)
{
    cv::Mat image;
    cv::Mat mask;
    std::string filename = "../resources/green_teapot_white_background/20220706_214554.jpg";
    cv::Rect rect = cv::Rect(3616, 1044, 1252, 1744); // Values of rect are manually determined with GIMP
    cv::Mat foregroundImage;

    image = cv::imread(filename);
    cv::resize(image, image, cv::Size(), 0.125, 0.125);
    rect.x *= 0.125;
    rect.y *= 0.125;
    rect.width *= 0.125;
    rect.height *= 0.125;
    ImagePreprocessor::doForegroundSegmentation(foregroundImage, mask, image, rect, 5);

    cv::imwrite("../resources/green_teapot_white_background/20220706_214554_foreground.jpg", foregroundImage);
}

void detectAndDrawMarkerForDataset(std::string inDir, std::string outDir)
{
    cv::Mat image;
    cv::Matx33d cameraMatrix(6005.641173008885, 0, 4030.950098307286, 0, 6002.681113514058, 2986.968236297804, 0, 0, 1);
    cv::Vec<double, 5> distCoeffs(0.08170529228771495, -0.2834249429739051, 0.0007430954776429432, 0.0006295724080059367, 0.3968821057473708);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat rVec;
    cv::Mat tVec;
    cv::Matx44d worldToCamera;
    PoseEstimator poseEstimator(cameraMatrix, distCoeffs);

    for (const auto &dirEntry : std::filesystem::recursive_directory_iterator(inDir))
    {
        image = cv::imread(dirEntry.path().string());
        poseEstimator.estimateMarkerPoses(ids, corners, rVec, tVec, image);
        cv::Mat imageWithDetectedMarkers = poseEstimator.drawArUcoMarkersAndAxisOnImage(image, ids, corners, rVec, tVec);
        std::string filename = dirEntry.path().filename().string();
        cv::imwrite(outDir + filename.substr(0, filename.find_last_of(".")) + "_markers.jpg", imageWithDetectedMarkers);

        worldToCamera = poseEstimator.computeCameraPoseMatrixFromBoardPose(rVec, tVec);
        std::cout << worldToCamera << std::endl;
    }
}

TEST(ImagePreprocessorTest, DetectTestBoard)
{
    detectAndDrawMarkerForDataset("../resources/green_teapot_brown_background/", "../resources/green_teapot_brown_background_preprocessed/");
}

TEST(ImagePreprocessorTest, ReadMultipleImagesAndComputeCameraPose)
{
    std::vector<std::filesystem::path> filepaths;
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> masks;
    std::vector<cv::Matx44d> cameraPoses;
    std::string inDir = "../resources/green_teapot_brown_background_subset/";
    std::string outDir = "../resources/green_teapot_brown_background_preprocessed/";
    // Values of rects are manually determined with GIMP
    std::unordered_map<std::string, cv::Rect> imageNameToRect{{"20220706_214805.jpg", cv::Rect(3364, 1004, 1848, 1476)}, {"20220706_214810.jpg", cv::Rect(3272, 608, 1412, 1560)}};
    cv::Matx33d cameraMatrix(6005.641173008885, 0, 4030.950098307286, 0, 6002.681113514058, 2986.968236297804, 0, 0, 1);
    cv::Vec<double, 5> distCoeffs(0.08170529228771495, -0.2834249429739051, 0.0007430954776429432, 0.0006295724080059367, 0.3968821057473708);
    ImagePreprocessor imPrep(cameraMatrix, distCoeffs);

#ifdef DO_FOREGROUND_SEGMENTATION
    imPrep.readImagesAndComputeCameraPoses(filepaths, images, cameraPoses, inDir, false, imageNameToRect, 2);
#else
    imPrep.readImagesAndComputeCameraPoses(filepaths, images, cameraPoses, inDir, true);
#endif

    for (unsigned int i = 0; i < std::min(filepaths.size(), images.size()); i++)
    {
        std::string filename = filepaths.at(i).filename().string();
#ifdef DO_FOREGROUND_SEGMENTATION
        std::string extension = "_foreground.jpg";
#else
        std::string extension = "_markers.jpg";
#endif
        cv::imwrite(outDir + filename.substr(0, filename.find_last_of(".")) + extension, images[i]);
        std::cout << cameraPoses[i] << std::endl;
    }
}
