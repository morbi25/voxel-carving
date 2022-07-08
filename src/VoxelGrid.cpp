#include "../inc/VoxelGrid.h"
#include "../inc/Eigen.h"
#include <fstream>
#include <iostream>

using namespace cv;
using namespace open3d;
using namespace std;

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