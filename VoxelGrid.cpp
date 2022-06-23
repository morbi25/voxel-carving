#include "VoxelGrid.h"

using namespace cv;

VoxelGrid::VoxelGrid(int sizeX_, int sizeY_, int sizeZ_)
{
	sizeX = sizeX_;
	sizeY = sizeY_;
	sizeZ = sizeZ_;

	grid = new uint8_t **[sizeZ];

	for (int z = 0; z < sizeZ; ++z)
	{
		grid[z] = new uint8_t *[sizeX];
		for (int x = 0; x < sizeX; ++x)
		{
			grid[z][x] = new uint8_t[sizeY];
			for (int y = 0; y < sizeY; ++y)
			{
				grid[z][x][y] = 1;
			}
		}
	}
}

VoxelGrid::~VoxelGrid()
{
	delete grid;
};

int VoxelGrid::getElement(int x, int y, int z)
{
	return (int)grid[z][x][y];
}

void VoxelGrid::setElement(int x, int y, int z, uint8_t val)
{
	if (x < sizeX && y < sizeY && z < sizeZ)
	{
		grid[z][x][y] = (val == 0) ? 0 : 1;
	}
}

void VoxelGrid::carveSingleImg(Mat img)
{
	Mat resizedImg;
	resize(img, resizedImg, Size(sizeX, sizeY));
	Mat grayImg(sizeX, sizeY, CV_8UC1);
	cvtColor(resizedImg, grayImg, COLOR_BGR2GRAY);

	for (int z = 0; z < sizeZ; ++z)
	{
		for (int x = 0; x < sizeX; ++x)
		{
			for (int y = 0; y < sizeY; ++y)
			{
				grid[z][x][y] = (grayImg.at<uint8_t>(x, y) == 0) ? 0 : 1;
			}
		}
	}
}

void VoxelGrid::render()
{
	Mat zAxis(sizeX, sizeY, CV_8UC1);
	Mat yAxis(sizeX, sizeZ, CV_8UC1);
	Mat xAxis(sizeY, sizeZ, CV_8UC1);

	for (int z = 0; z < sizeZ; ++z)
	{
		for (int x = 0; x < sizeX; ++x)
		{
			for (int y = 0; y < sizeY; ++y)
			{
				zAxis.at<uint8_t>(x, y) = 255;
				yAxis.at<uint8_t>(x, z) = 255;
				xAxis.at<uint8_t>(y, z) = 255;
			}
		}
	}

	for (int z = 0; z < sizeZ; ++z)
	{
		for (int x = 0; x < sizeX; ++x)
		{
			for (int y = 0; y < sizeY; ++y)
			{
				zAxis.at<uint8_t>(x, y) = (grid[z][x][y] == 1) ? 0 : 1 * zAxis.at<uint8_t>(x, y);
				yAxis.at<uint8_t>(x, z) = (grid[z][x][y] == 1) ? 0 : 1 * yAxis.at<uint8_t>(x, z);
				xAxis.at<uint8_t>(y, z) = (grid[z][x][y] == 1) ? 0 : 1 * xAxis.at<uint8_t>(y, z);
			}
		}
	}

	// Invert the pictures.

	for (int x = 0; x < sizeX; ++x)
	{
		for (int y = 0; y < sizeY; ++y)
		{
			zAxis.at<uint8_t>(x, y) = (zAxis.at<uint8_t>(x, y) == 255) ? 0 : 255;
		}
	}

	for (int x = 0; x < sizeX; ++x)
	{
		for (int z = 0; z < sizeZ; ++z)
		{
			yAxis.at<uint8_t>(x, z) = (yAxis.at<uint8_t>(x, z) == 255) ? 0 : 255;
		}
	}

	for (int y = 0; y < sizeY; ++y)
	{
		for (int z = 0; z < sizeZ; ++z)
		{
			xAxis.at<uint8_t>(y, z) = (xAxis.at<uint8_t>(y, z) == 255) ? 0 : 255;
		}
	}

	// Resize the pictures to the 50x to get sharper images.
	int resize_factor = 50;
	Mat zAxisResized(resize_factor * sizeX, resize_factor * sizeY, CV_8UC1);
	Mat yAxisResized(resize_factor * sizeX, resize_factor * sizeZ, CV_8UC1);
	Mat xAxisResized(resize_factor * sizeY, resize_factor * sizeZ, CV_8UC1);

	resize(zAxis, zAxisResized, Size(resize_factor * sizeX, resize_factor * sizeY), 0, 0, INTER_AREA);
	resize(yAxis, yAxisResized, Size(resize_factor * sizeX, resize_factor * sizeZ), 0, 0, INTER_AREA);
	resize(xAxis, xAxisResized, Size(resize_factor * sizeY, resize_factor * sizeZ), 0, 0, INTER_AREA);

	imshow("Z Axis", zAxisResized);
	imshow("Y Axis", yAxisResized);
	imshow("X Axis", xAxisResized);

	waitKey(0);
}