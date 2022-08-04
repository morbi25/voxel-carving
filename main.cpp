#include <opencv2/opencv.hpp>

#include "inc/ImagePreprocessor.hpp"
#include "inc/VoxelGrid.hpp"

int main(int argc, char** argv)
{
    // Define a voxel grid of size 400x400x200 with step size 0.0007
    // For the teapot and the bunny datasets
    VoxelGrid voxelGrid(400, 400, 200, 0, 0, 0, 0.0007);

    // Define a voxel grid of size 200x200x300 with step size 0.0014
    // For the phone booth dataset
    //VoxelGrid voxelGrid(200, 200, 300, 0, 0, 0, 0.0014);

    // Input directory for the voxel carving algorithm
    std::string inDir = "../resources/green_bunny/";

    // Define camera intrinsics and distortion coefficients estimated by the doCalibration procedure
    cv::Matx33d cameraMatrix(6005.641173008885, 0, 4030.950098307286, 0, 6002.681113514058, 2986.968236297804, 0, 0, 1);
    cv::Vec<double, 5> distCoeffs(0.08170529228771495, -0.2834249429739051, 0.0007430954776429432, 0.0006295724080059367, 0.3968821057473708);

    // Define pose estimator
    PoseEstimator poseEstimator(cameraMatrix, distCoeffs);
    
    // SEGMENTER OPTION 1: Define GrabCut as a foreground segmenter
    //GrabCut foregroundSegmenter;

    // SEGMENTER OPTION 2: Define ColorThreshold as a foreground segmenter
    //ColorThreshold foregroundSegmenter;

    // SEGMENTER OPTION 3: Define U2Net as a foreground segmenter
    U2Net foregroundSegmenter;

    // Define image preprocessor with pre-defined pose estimator and foreground segmenter
    ImagePreprocessor imagePreprocessor(poseEstimator, foregroundSegmenter);

    // Get images and its meta data
    std::vector<ImageMeta> imageMetas = imagePreprocessor.readImagesAndComputeCameraPoses(inDir);

    // Print additional information
    // std::string outDir = "../resources/green_bunny_v2_preprocessed_grabcut/";
    // imagePreprocessor.verbose(imageMetas, outDir, false, true, false);

    // Perform voxel carving
    voxelGrid.carve(imageMetas, 0.125, 0.1);

    // Render the output voxel grid after carving in Open3D
    voxelGrid.render();

    return 0;
}