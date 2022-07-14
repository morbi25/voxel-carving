#include <stdio.h>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "inc/Eigen.h"
#include "inc/ImagePreprocessor.hpp"
#include "src/calib_intrinsic.cpp"
#include "inc/VoxelGrid.h"
#include "open3d/Open3D.h"

int main(int argc, char **argv)
{
    // NILS's part
    VoxelGrid voxelGrid(400, 400, 100, 0, 0, 0, 0.0007);

    std::vector<std::filesystem::path> filepaths;
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> results;
    std::vector<cv::Matx44d> cameraPoses;
    std::string inDir = "../resources/green_teapot_brown_background_subset/";
    std::string outDir = "../resources/green_teapot_brown_background_preprocessed/";

    // Values of rects are manually determined with GIMP
    std::unordered_map<std::string, cv::Rect> imageNameToRect{
        {"20220706_214805.jpg", cv::Rect(3364, 1004, 1848, 1476)},
        {"20220706_214810.jpg", cv::Rect(3272, 608, 1412, 1560)},
        {"20220706_214815.jpg", cv::Rect(3070, 677, 17433, 1403)},
        {"20220706_214821.jpg", cv::Rect(2805, 888, 2121, 1213)},
        {"20220706_214826.jpg", cv::Rect(3405, 901, 1671, 1592)},
        {"20220706_214831.jpg", cv::Rect(3468, 683, 1333, 1656)},
        {"20220706_214838.jpg", cv::Rect(3299, 808, 1505, 1391)},
        {"20220706_214844.jpg", cv::Rect(2816, 1165, 2245, 1322)},
        {"20220706_214849.jpg", cv::Rect(2931, 1425, 2501, 2073)},
        {"20220706_214856.jpg", cv::Rect(3437, 1655, 2481, 2617)},
        {"20220706_214901.jpg", cv::Rect(3477, 2245, 2273, 2485)},
        {"20220706_214905.jpg", cv::Rect(2641, 1657, 2241, 2589)},
        {"20220706_214913.jpg", cv::Rect(3529, 1909, 2117, 2293)},
        {"20220706_214959.jpg", cv::Rect(2913, 2380, 2881, 3404)},
    };
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

    // for loop over images
    for (int i = 0; i < images.size(); i++)
    {
        cv::Mat image1C = images[i];
        cv::Mat image1(1000, 750, CV_8UC1);
        cv::cvtColor(image1C, image1, cv::COLOR_BGR2GRAY);
        results.push_back(image1);
    }

    voxelGrid.carve(results, cameraPoses, images, 0.125, 0.1);

    for (int i = 0; i < images.size(); i++)
    {
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Display Image", images[i]);
        cv::waitKey(0);
    }

    voxelGrid.render();

    return 0;
}