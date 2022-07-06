#include "VoxelGrid.h"
#include "open3d/Open3D.h"
#include "inc/Eigen.h"
#include <fstream>
#include <iostream>

using namespace cv;
using namespace open3d;
using namespace std;

VoxelGrid::VoxelGrid(int sizeX_, int sizeY_, int sizeZ_, float startX_, float startY_, float startZ_, float step_)
{
	// Dimensions of the voxel grid -- Decides number of voxels we have in the voxel grid
	sizeX = sizeX_;
	sizeY = sizeY_;
	sizeZ = sizeZ_;

	// World coordinates of the corner of the voxel grid -- Decides where the voxel grid is placed in the world
	startX = startX_;
	startY = startY_;
	startZ = startZ_;

	// Spacing in between voxels in the world coordnates -- Decides how big the voxel grid is in the world
	step = step_;

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

void VoxelGrid::toPLY()
{
	ofstream myfile;
	myfile.open("voxel_grid.ply");
	int count = 0;
	for (int z = 0; z < sizeZ; ++z)
	{
		for (int x = 0; x < sizeX; ++x)
		{
			for (int y = 0; y < sizeY; ++y)
			{
				if (grid[z][x][y] == 1)
					count++;
			}
		}
	}

	myfile << "ply" << endl;
	myfile << "format ascii 1.0" << endl;
	myfile << "comment Created by Open3D" << endl;
	// myfile << "element origin 1" << endl;
	// myfile << "property double x" << endl;
	// myfile << "property double y" << endl;
	// myfile << "property double z" << endl;
	myfile << "element voxel_size 1" << endl;
	myfile << "property double val" << endl;
	myfile << "element vertex " << count << endl;
	myfile << "property double x" << endl;
	myfile << "property double y" << endl;
	myfile << "property double z" << endl;
	myfile << "property uchar red" << endl;
	myfile << "property uchar green" << endl;
	myfile << "property uchar blue" << endl;
	myfile << "end_header" << endl;

	// myfile << 0 << " " << 0 << " " << 0 << endl;
	myfile << 0.5 << endl;

	for (int z = 0; z < sizeZ; ++z)
	{
		for (int x = 0; x < sizeX; ++x)
		{
			for (int y = 0; y < sizeY; ++y)
			{
				if (grid[z][x][y] == 1)
				{
					myfile << x << " " << y << " " << z << " " << 0 << " " << 255 << " " << 0 << endl;
				}
			}
		}
	}

	myfile.close();
}

void VoxelGrid::render()
{
	VoxelGrid::toPLY();
	// create new VoxelGrid object
	auto vg = std::make_shared<geometry::VoxelGrid>();
	io::ReadVoxelGrid("voxel_grid.ply", *vg);
	open3d::visualization::DrawGeometries({vg});
}


cv::Point3f VoxelGrid::voxelToWorld(int x, int y, int z)
{
	cv::Point3f point;
	point.x = startX + step * x;
	point.y = startY + step * y;
	point.z = startZ + step * z;

	return point;
}

cv::Point2i VoxelGrid::projectVoxel(int x, int y, int z, cv::Mat P)
{
	cv::Point3f voxelWorld = voxelToWorld(x, y, z);
	cv::Point2i point;
	float zc = P.at<float>(2, 0) * voxelWorld.x + P.at<float>(2, 1) * voxelWorld.y + P.at<float>(2, 2) * voxelWorld.z + P.at<float>(2, 3);
	//float zc = 1.0;

	point.x = (P.at<float>(0, 0) * voxelWorld.x + P.at<float>(0, 1) * voxelWorld.y + P.at<float>(0, 2) * voxelWorld.z + P.at<float>(0, 3)) / zc;
	point.y = (P.at<float>(1, 0) * voxelWorld.x + P.at<float>(1, 1) * voxelWorld.y + P.at<float>(1, 2) * voxelWorld.z + P.at<float>(1, 3)) / zc;

	return point;
}

void VoxelGrid::carve(std::vector<cv::Mat> images, std::vector<cv::Mat> PMatrices, float voteTreshold)
{
	int numImages = images.size();
	// Looping over voxels
	for (int x = 0; x < sizeX; ++x)
	{
		for (int y = 0; y < sizeY; ++y)
		{
			for (int z = 0; z < sizeZ; ++z)
			{
				int vote = 0;

				// Looping over images
				for (int i = 0; i < numImages; ++i)
				{
					// One projection matrix from world to pixel coordinates per image
					cv::Mat P = PMatrices[i];
					
					// Input images need to be grayscale CV_8UC1
					cv::Mat image = images[i];

					// Project voxel onto image
					cv::Point2i projectedV = projectVoxel(x, y, z, P);

					if (projectedV.x < image.cols && projectedV.y < image.rows)
					{
						// Vote to carve the voxel if projects to background pixel
						if (image.at<uint8_t>(projectedV.x, projectedV.y) == 0)
						{
							++vote;
						}
					}
				}
				// If enough votes to carve the voxel (1.0 = all votes needed)
				if (vote >= voteTreshold * numImages)
				{
					setElement(x, y, z, 0);
				}
			}
		}
	}
}
