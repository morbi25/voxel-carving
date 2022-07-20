#include <stdio.h>
#include <filesystem>
#include <opencv2/opencv.hpp>

#include "inc/Eigen.h"
#include "inc/ImagePreprocessor.hpp"
#include "src/calib_intrinsic.cpp"
#include "inc/VoxelGrid.h"
#include "open3d/Open3D.h"

int main(int argc, char** argv)
{
    // NILS's part
    VoxelGrid voxelGrid(400, 400, 150, 0, 0, 0, 0.0007);

    std::vector<std::filesystem::path> filepaths;
    std::vector<cv::Mat> images;
    std::vector<cv::Mat> results;
    std::vector<cv::Matx44d> cameraPoses;

    std::string inDir = "../resources/green_teapot_v2/";
    std::string outDir = "../resources/green_teapot_v2_preprocessed/";

    std::unordered_map<std::string, cv::Rect> imageNameToRect{
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
        for (int j = 0; j < image1C.rows; j++)
        {
            for (int k = 0; k < image1C.cols; k++)
            {
                cv::Vec3b pixel = image1C.at<cv::Vec3b>(j, k);
                double BRTreshold = pixel.val[1] * 1.3;
                if (pixel.val[1] < 40 || (double)pixel.val[0] + (double)pixel.val[2] > BRTreshold)
                {
                    image1C.at<cv::Vec3b>(j, k) = cv::Vec3b(0, 0, 0);
                }
            }
        }

        cv::Mat image1(1000, 750, CV_8UC1);
        cv::cvtColor(image1C, image1, cv::COLOR_BGR2GRAY);
        results.push_back(image1);
    }

    voxelGrid.carve(results, cameraPoses, images, 0.125, 0.1);

    // for (int i = 0; i < images.size(); i++)
    // {
    //     cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
    //     cv::imshow("Display Image", images[i]);
    //     cv::waitKey(0);
    // }

    voxelGrid.toPLY();
    // voxelGrid.render();

    return 0;
}