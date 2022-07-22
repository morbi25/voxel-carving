#include <opencv2/opencv.hpp>

#include "inc/ImagePreprocessor.hpp"
#include "inc/VoxelGrid.h"

int main(int argc, char **argv)
{
    // Define a voxel grid of size 400x400x150 with step size 0.0007
    VoxelGrid voxelGrid(400, 400, 150, 0, 0, 0, 0.0007);

    // Input directory for the voxel carving algorithm
    std::string inDir = "../resources/green_teapot_v2/";

    /*std::unordered_map<std::string, cv::Rect> imageNameToRect{
        {"20220713_205043.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205047.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205052.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205056.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205101.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205106.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205111.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205115.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205121.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205129.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205134.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205141.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205145.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205151.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205157.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205202.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205210.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205220.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205224.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205230.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205236.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205244.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205249.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205255.jpg", cv::Rect(0, 0, 7990, 5990)},
        {"20220713_205301.jpg", cv::Rect(0, 0, 7990, 5990)},
    };*/

    // Define camera intrinsics and distortion coefficients estimated by the doCalibration procedure
    cv::Matx33d cameraMatrix(6005.641173008885, 0, 4030.950098307286, 0, 6002.681113514058, 2986.968236297804, 0, 0, 1);
    cv::Vec<double, 5> distCoeffs(0.08170529228771495, -0.2834249429739051, 0.0007430954776429432, 0.0006295724080059367, 0.3968821057473708);
    
    // Define pose estimator and a specific foreground segmenter
    PoseEstimator poseEstimator(cameraMatrix, distCoeffs);
    ColorThreshold foregroundSegmenter;

    // Define image preprocessor with pre-defined pose estimator and foreground segmenter
    ImagePreprocessor imagePreprocessor(poseEstimator, foregroundSegmenter);

    // Get images and its meta data
    std::vector<ImageMeta> imageMetas = imagePreprocessor.readImagesAndComputeCameraPoses(inDir);
    
    // Perform voxel carving
    voxelGrid.carve(imageMetas, 0.125, 0.1);

    /*for (int i = 0; i < images.size(); i++)
    {
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", images[i]);
        cv::waitKey(0);
    }*/

    // voxelGrid.toPLY();

    // Renfer the output voxel grid after carving in Open3D
    voxelGrid.render();

    return 0;
}