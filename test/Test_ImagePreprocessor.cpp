#include <filesystem>
#include <opencv2/opencv.hpp>
#include "gtest/gtest.h"

#include "../inc/ImagePreprocessor.hpp"

TEST(ImagePreprocessorTest, ReadSingleImageAndExtractForeground)
{
    std::string filename = "../resources/green_bunny/20220713_204828.jpg";

    cv::Mat image = cv::imread(filename);
    ColorThreshold colorThreshold;
    cv::Mat foregroundImage = colorThreshold.doForegroundSegmentation(image);

    cv::imwrite("../resources/20220713_204828_foreground.jpg", foregroundImage);
}

/*void detectAndDrawMarkerForDataset(std::string inDir, std::string outDir)
{
    ImageMeta imageMeta;
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
        imageMeta.filepath = dirEntry.path();
        imageMeta.image = cv::imread(imageMeta.filepath.string());
        poseEstimator.detectMarkersAndEstimateBoardPose(ids, corners, rVec, tVec, imageMeta);
        cv::Mat imageWithDetectedMarkers = poseEstimator.drawArUcoMarkersAndAxisOnImage(imageMeta.image, ids, corners, rVec, tVec);
        std::string filename = dirEntry.path().filename().string();
        cv::imwrite(outDir + filename.substr(0, filename.find_last_of(".")) + "_markers.jpg", imageWithDetectedMarkers);

        worldToCamera = poseEstimator.computeCameraPoseMatrixFromBoardPose(rVec, tVec);
        std::cout << worldToCamera << std::endl;
    }
}*/

TEST(ImagePreprocessorTest, DetectTestBoard)
{
    ImageMeta imageMeta;
    cv::Matx33d cameraMatrix(6005.641173008885, 0, 4030.950098307286, 0, 6002.681113514058, 2986.968236297804, 0, 0, 1);
    cv::Vec<double, 5> distCoeffs(0.08170529228771495, -0.2834249429739051, 0.0007430954776429432, 0.0006295724080059367, 0.3968821057473708);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat rVec;
    cv::Mat tVec;
    cv::Matx44d worldToCamera;
    PoseEstimator poseEstimator(cameraMatrix, distCoeffs);
        
    imageMeta.filepath = "../resources/green_bunny/20220713_204828.jpg";
    imageMeta.image = cv::imread(imageMeta.filepath.string());
    poseEstimator.detectMarkersAndEstimateBoardPose(ids, corners, rVec, tVec, imageMeta);
    cv::Mat imageWithDetectedMarkers = poseEstimator.drawArUcoMarkersAndAxisOnImage(imageMeta.image, ids, corners, rVec, tVec);
    cv::imwrite("../resources/20220713_204828_markers.jpg", imageWithDetectedMarkers);

    worldToCamera = poseEstimator.computeCameraPoseMatrixFromBoardPose(rVec, tVec);
    std::cout << worldToCamera << std::endl;
}

TEST(ImagePreprocessorTest, ReadMultipleImagesAndComputeCameraPose)
{
    std::vector<cv::Matx44d> cameraPoses;
    std::string inDir = "../resources/green_bunny/";
    std::string outDir = "../resources/";
    cv::Matx33d cameraMatrix(6005.641173008885, 0, 4030.950098307286, 0, 6002.681113514058, 2986.968236297804, 0, 0, 1);
    cv::Vec<double, 5> distCoeffs(0.08170529228771495, -0.2834249429739051, 0.0007430954776429432, 0.0006295724080059367, 0.3968821057473708);
    PoseEstimator poseEstimator(cameraMatrix, distCoeffs);
    ColorThreshold foregroundSegmenter;
    ImagePreprocessor imPrep(poseEstimator, foregroundSegmenter);

    std::vector<ImageMeta> imageMetas = imPrep.readImagesAndComputeCameraPoses(inDir, true);
    imPrep.verbose(imageMetas, outDir, false, false, true);
}
