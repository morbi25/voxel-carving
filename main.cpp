#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "inc/Eigen.h"
#include "calib_intrinsic.cpp"
#include "VoxelGrid.h"

using namespace cv;

int main(int argc, char **argv)
{
    // do_calib("C:/Users/Alex/Documents/Alexander/Studium/Master_TUM/Studium/SS22/3D Scanning and Motion Capture/Exercises/voxel-carving/calib_conf.xml", 11);
    cv::Mat cameraMatrix, distCoeffs, R, T;
    do_calib(cameraMatrix, distCoeffs, R, T);

    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;
    std::cout << "Rotation vector : " << R << std::endl;
    std::cout << "Translation vector : " << T << std::endl;

    // // Testing Eigen
    // Eigen::Vector3f v(1, 2, 3);
    // // Print the vector
    // std::cout << "v = " << v << std::endl;

    // // Testing OpenCV
    // if (argc != 2)
    // {
    //     printf("usage: DisplayImage.out <Image_Path>\n");
    //     return -1;
    // }
    // Mat image;
    // image = imread(argv[1], 1);
    // if (!image.data)
    // {
    //     printf("No image data \n");
    //     return -1;
    // }
    // namedWindow("Display Image", WINDOW_AUTOSIZE);
    // imshow("Display Image", image);
    // waitKey(0);

    // Testing VoxelGrid
    VoxelGrid voxelGrid(10, 10, 10);
    voxelGrid.setElement(0, 0, 0, 0);
    voxelGrid.setElement(0, 0, 1, 0);
    voxelGrid.setElement(0, 0, 2, 0);
    voxelGrid.setElement(0, 0, 3, 0);
    voxelGrid.setElement(0, 0, 4, 0);
    voxelGrid.setElement(0, 0, 5, 0);
    voxelGrid.setElement(0, 0, 6, 0);
    voxelGrid.setElement(0, 0, 7, 0);
    voxelGrid.setElement(0, 0, 8, 0);
    voxelGrid.setElement(0, 0, 9, 0);

    // voxelGrid.setElement(1, 1, 1, 0);
    // voxelGrid.setElement(2, 2, 2, 0);
    // voxelGrid.setElement(3, 3, 3, 0);
    // voxelGrid.setElement(4, 4, 4, 0);
    // voxelGrid.setElement(5, 5, 5, 1);
    // voxelGrid.setElement(6, 6, 6, 1);
    // voxelGrid.setElement(7, 7, 7, 1);
    // voxelGrid.setElement(8, 8, 8, 1);
    // voxelGrid.setElement(9, 9, 9, 1);
    voxelGrid.render();
    return 0;
}