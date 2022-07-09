#include "../inc/VoxelGrid.h"
#include "../inc/Eigen.h"
#include <fstream>
#include <iostream>

using namespace cv;
using namespace open3d;
using namespace std;

VoxelGrid::VoxelGrid(int sizeX_, int sizeY_, int sizeZ_, double startX_, double startY_, double startZ_, double step_)
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

	std::cout << "EXPORT STARTING, NUMBER OF VOXELS:" << count << endl;

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
		std::cout << "Exporting Z:" << z << endl;
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

Point3d VoxelGrid::voxelToWorld(int x, int y, int z)
{
	Point3d point;
	point.x = startX + double(step * x);
	point.y = startY + double(step * y);
	point.z = startZ + double(step * z);

	return point;
}

Point2i VoxelGrid::projectVoxel(int x, int y, int z, Matx44d pose, double imgScale)
{
	Matx34d standardProjection(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);

	Matx33d intrinsic(
		6005.641173008885, 0, 4030.950098307286,
		0, 6002.681113514058, 2986.968236297804,
		0, 0, 1);

	Matx33d intrinsicScaled = intrinsic * imgScale;

	// General projection matrix from world to camera to screen
	Matx34d P = intrinsicScaled * (standardProjection * pose);

	// 2D pixel point on screen
	Point2i pointPixel;

	// 3D world coordinates of the voxel
	Point3d voxelWorld = voxelToWorld(x, y, z);

	// 4D world coordinates of the voxel (3D, 1)
	Matx41d voxelWorld4d(voxelWorld.x, voxelWorld.y, voxelWorld.z, 1.0);

	Matx31d Pixel3d = P * voxelWorld4d;

	pointPixel.x = Pixel3d(0, 0) / Pixel3d(2, 0);
	pointPixel.y = Pixel3d(1, 0) / Pixel3d(2, 0);

	return pointPixel;
}

void VoxelGrid::carve(std::vector<Mat> images, std::vector<Matx44d> poses, std::vector<Mat> results, double imgScale, float voteTreshold)
{
	int numImages = images.size();
	// Looping over voxels
	for (int x = 0; x < sizeX; ++x)
	{
		std::cout << "Carving X:" << x << std::endl;
		for (int y = 0; y < sizeY; ++y)
		{
			for (int z = 0; z < sizeZ; ++z)
			{
				int vote = 0;

				// Looping over images
				for (int i = 0; i < numImages; ++i)
				{
					// One projection matrix from world to screen coordinates per image
					Matx44d pose = poses[i];

					// Input images need to be grayscale CV_8UC1
					Mat image = images[i];

					// Project voxel onto image
					cv::Point2i projectedV = projectVoxel(x, y, z, pose, imgScale);

					if (projectedV.x < image.cols && projectedV.y < image.rows && projectedV.x > -1 && projectedV.y > -1)
					{
						if (x == 0 && y == 0 && z == 0)
						{
							circle(results[i], projectedV, 7, Scalar(51, 255, 255), -1);
						}
						else if (y == 0 && z == 0) // X
						{
							circle(results[i], projectedV, 3, Scalar(0, 0, 255), -1);
						}
						else if (x == 0 && z == 0) // y
						{
							circle(results[i], projectedV, 3, Scalar(0, 255, 0), -1);
						}
						else if (x == 0 && y == 0) // z
						{
							circle(results[i], projectedV, 3, Scalar(255, 0, 0), -1);
						}

						circle(results[i], projectedV, 0, Scalar(255, 0, 255), -1);
						// Vote to carve the voxel if projects to background pixel
						if (image.at<uint8_t>(projectedV.y, projectedV.x) == 0)
						{
							/*
							std::stringstream ss;
							ss << x << "," << y << "," << z;
							std::string s = ss.str();

							putText(results[i], s, projectedV, cv::FONT_HERSHEY_DUPLEX, 0.2, Scalar(128, 128, 128));
							*/
							circle(results[i], projectedV, 0, Scalar(128, 128, 128), -1);
							++vote;
						}
					}
				}
				// If enough votes to carve the voxel (1.0 = all votes needed)
				if (vote >= voteTreshold * numImages)
				{
					setElement(x, y, z, 0);
					// std::cout << "carved at:"<<  x << ", " << y << ", " << z << endl;
				}
			}
		}
	}
}
