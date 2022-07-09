#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "inc/Eigen.h"
#include "src/calib_intrinsic.cpp"
#include "inc/VoxelGrid.h"
#include "open3d/Open3D.h"

using namespace cv;

int main(int argc, char **argv)
{
    VoxelGrid voxelGrid(200, 200, 200, 0.72, -1.3, 0.00, 0.0003);

    std::vector<Mat> images;
    std::vector<Mat> results;
    std::vector<Matx44d> poses;

    Mat image1C = imread("..\\resources\\Examples\\20220706_214805_foreground_fix.jpg");
    Mat image1(1000, 750, CV_8UC1);
    cvtColor(image1C, image1, COLOR_BGR2GRAY);

    Mat image2C = imread("..\\resources\\Examples\\20220706_214810_foreground_fix.jpg");
    Mat image2(1000, 750, CV_8UC1);
    cvtColor(image2C, image2, COLOR_BGR2GRAY);

    //images.push_back(image1);
    //results.push_back(image1C);
    images.push_back(image2);
    results.push_back(image2C);

    Matx44d pose1(
        -0.7731083419143783, -0.5281222579736478, 0.3512696575214211, -0.123085193094924,
        -0.6325915190749175, 0.6017026575313881, -0.4876288362209038, 0.4701735189239425,
        0.04616775559726605, -0.5992001272967828, -0.799267005318392, 0.3446566037210394,
        0, 0, 0, 1);

    Matx44d pose2(
        -0.9974029574492606, 0.03003503696173567, -0.06546172183937367, 0.1561629506843841,
        0.06222039443917605, 0.8171126077748148, -0.5731104681745707, 0.5122759528158224,
        0.03627620414681929, -0.5756951300559606, -0.8168593234104322, 0.36288435310979,
        0, 0, 0, 1);
    
    //poses.push_back(pose1);
    poses.push_back(pose2);

    voxelGrid.carve(images, poses, results, 0.125);
    //voxelGrid.toPLY();

    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", results[0]);
    waitKey(0);

    return 0;
}