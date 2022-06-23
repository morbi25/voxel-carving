#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "inc/Eigen.h"
using namespace cv;

int main(int argc, char **argv)
{
    // Testing Eigen
    Eigen::Vector3f v(1, 2, 3);
    // Print the vector
    std::cout << "v = " << v << std::endl;

    // Testing OpenCV
    if (argc != 2)
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    Mat image;
    image = imread(argv[1], 1);
    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", image);
    waitKey(0);
    return 0;
}