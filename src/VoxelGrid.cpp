#include <fstream>
#include <iostream>

#include "../inc/ImagePreprocessor.hpp"
#include "../inc/VoxelGrid.hpp"
#include "../inc/Eigen.h"
#include <omp.h>

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
	colorGrid = new cv::Vec3b * *[sizeZ];

	for (int z = 0; z < sizeZ; ++z)
	{
		grid[z] = new uint8_t *[sizeX];
		colorGrid[z] = new cv::Vec3b * [sizeX];
		for (int x = 0; x < sizeX; ++x)
		{
			grid[z][x] = new uint8_t[sizeY];
			colorGrid[z][x] = new cv::Vec3b[sizeY];

			for (int y = 0; y < sizeY; ++y)
			{
				grid[z][x][y] = 1;
				colorGrid[z][x][y] = cv::Vec3b(0, 0, 0);
			}
		}
	}
}

VoxelGrid::~VoxelGrid()
{
	delete grid;
	delete colorGrid;
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

cv::Vec3b VoxelGrid::getElementColor(int x, int y, int z)
{
	return colorGrid[z][x][y];
}

void VoxelGrid::setElementColor(int x, int y, int z, cv::Vec3b rgbColor)
{
	colorGrid[z][x][y] = rgbColor;
}

void VoxelGrid::toPLY()
{
	std::ofstream myfile;
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

	// std::cout << "EXPORT STARTING, NUMBER OF VOXELS:" << count << endl;

	myfile << "ply" << std::endl;
	myfile << "format ascii 1.0" << std::endl;
	myfile << "comment Created by Open3D" << std::endl;
	// myfile << "element origin 1" << std::endl;
	// myfile << "property double x" << std::endl;
	// myfile << "property double y" << std::endl;
	// myfile << "property double z" << std::endl;
	myfile << "element voxel_size 1" << std::endl;
	myfile << "property double val" << std::endl;
	myfile << "element vertex " << count << std::endl;
	myfile << "property double x" << std::endl;
	myfile << "property double y" << std::endl;
	myfile << "property double z" << std::endl;
	myfile << "property uchar red" << std::endl;
	myfile << "property uchar green" << std::endl;
	myfile << "property uchar blue" << std::endl;
	myfile << "end_header" << std::endl;

	// myfile << 0 << " " << 0 << " " << 0 << endl;
	myfile << 0.5 << std::endl;

	for (int z = 0; z < sizeZ; ++z)
	{
		// std::cout << "Exporting Z:" << z << endl;
		for (int x = 0; x < sizeX; ++x)
		{
			for (int y = 0; y < sizeY; ++y)
			{
				if (grid[z][x][y] == 1)
				{
					cv::Vec3b voxelColor = getElementColor(x, y, z);
					myfile << x << " " << y << " " << z << " " << (int)voxelColor.val[0] << " " << (int)voxelColor.val[1] << " " << (int)voxelColor.val[2] << std::endl;
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
	auto vg = std::make_shared<open3d::geometry::VoxelGrid>();
	open3d::io::ReadVoxelGrid("voxel_grid.ply", *vg);
	open3d::visualization::DrawGeometries({vg});
}

cv::Point3d VoxelGrid::voxelToWorld(int x, int y, int z)
{
	cv::Point3d point;
	point.x = startX + double(step * x);
	point.y = startY + double(step * y);
	point.z = startZ + double(step * z);

	return point;
}

cv::Point2i VoxelGrid::projectVoxel(int x, int y, int z, cv::Matx44d pose, double imgScale)
{
	cv::Matx34d standardProjection(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);

	cv::Matx33d intrinsicScaled(
		6005.641173008885 * imgScale, 0, 4030.950098307286 * imgScale,
		0, 6002.681113514058 * imgScale, 2986.968236297804 * imgScale,
		0, 0, 1);

	// General projection matrix from world to camera to screen
	cv::Matx34d P = intrinsicScaled * (standardProjection * pose);

	// 2D pixel point on screen
	cv::Point2i pointPixel;

	// 3D world coordinates of the voxel
	cv::Point3d voxelWorld = voxelToWorld(x, y, z);

	// 4D world coordinates of the voxel (3D, 1)
	cv::Matx41d voxelWorld4d(voxelWorld.x, voxelWorld.y, voxelWorld.z, 1.0);

	cv::Matx31d Pixel3d = P * voxelWorld4d;

	pointPixel.x = Pixel3d(0, 0) / Pixel3d(2, 0);
	pointPixel.y = Pixel3d(1, 0) / Pixel3d(2, 0);

	return pointPixel;
}

void VoxelGrid::carve(std::vector<ImageMeta> imageMetas, double imgScale, float voteThreshold)

{
	int numImages = imageMetas.size();

// Looping over voxels
#pragma omp parallel for
	for (int x = 0; x < sizeX; ++x)
	{
#pragma omp parallel for // benchmark again
		// std::cout << "Carving X:" << x << std::endl;
		for (int y = 0; y < sizeY; ++y)
		{
			for (int z = 0; z < sizeZ; ++z)
			{
				int vote = 0;
				std::vector<cv::Vec3b> voxelColorProposals;

				// Looping over images
				for (int i = 0; i < numImages; ++i)
				{
					// One projection matrix from world to screen coordinates per image
					cv::Matx44d pose = imageMetas[i].cameraPose;

					// Input images need to be grayscale CV_8UC1
					cv::Mat image = imageMetas[i].foregroundImage;

					// Project voxel onto image
					cv::Point2i projectedV = projectVoxel(x, y, z, pose, imgScale);

					if (projectedV.x < image.cols && projectedV.y < image.rows && projectedV.x > -1 && projectedV.y > -1)
					{
#ifdef DO_GRID_VISUALIZATION
						// Draw START CORNER (yellow), X (red), Y (green), Z (blue) axis of the grid
						cv::Mat results = imageMetas[i].image;
						if (x == 0 && y == 0 && z == 0) // START CORNER
						{
							circle(results, projectedV, 7, cv::Scalar(51, 255, 255), -1);
						}
						else if (y == 0 && z == 0) // X
						{
							circle(results, projectedV, 3, cv::Scalar(0, 0, 255), -1);
						}
						else if (x == 0 && z == 0) // Y
						{
							circle(results, projectedV, 3, cv::Scalar(0, 255, 0), -1);
						}
						else if (x == 0 && y == 0) // Z
						{
							circle(results, projectedV, 3, cv::Scalar(255, 0, 0), -1);
						}

						// Draw voxels in the grid that project to foregroud (purple) (Dont hide axis)
						cv::Vec3b prevPixel = results.at<cv::Vec3b>(projectedV.y, projectedV.x);
						if (!(prevPixel.val[0] == 0 && prevPixel.val[1] == 0 && prevPixel.val[2] == 255) && !(prevPixel.val[0] == 0 && prevPixel.val[1] == 255 && prevPixel.val[2] == 0) && !(prevPixel.val[0] == 255 && prevPixel.val[1] == 0 && prevPixel.val[2] == 0))
						{
							circle(results, projectedV, 0, cv::Scalar(255, 0, 255), -1);
						}

#endif
						// Vote to carve the voxel if projects to background pixel
						cv::Vec3b imagePixel = image.at<cv::Vec3b>(projectedV.y, projectedV.x);
						if (imagePixel.val[0] == 0 && imagePixel.val[1] == 0 && imagePixel.val[2] == 0)
						{
#ifdef DO_GRID_VISUALIZATION
							// Draw voxels in the grid that project to backgroud (gray) (Dont hide axis)
							if (!(prevPixel.val[0] == 0 && prevPixel.val[1] == 0 && prevPixel.val[2] == 255) && !(prevPixel.val[0] == 0 && prevPixel.val[1] == 255 && prevPixel.val[2] == 0) && !(prevPixel.val[0] == 255 && prevPixel.val[1] == 0 && prevPixel.val[2] == 0))
							{
								circle(results, projectedV, 0, cv::Scalar(128, 128, 128), -1);
							}
#endif
							++vote;
						}
						else
						{
							voxelColorProposals.push_back(imagePixel);
						}
					}
				}
				// If enough votes to carve the voxel (1.0 = all votes needed)
				if (vote >= voteThreshold * numImages)
				{
					setElement(x, y, z, 0);
				}
				else
				{
					int sumR = 0;
					int sumG = 0;
					int sumB = 0;
					int numProposals = voxelColorProposals.size();
					if (numProposals == 0) numProposals = 1;
					for (cv::Vec3b colorProp : voxelColorProposals)
					{
						sumR += colorProp.val[0];
						sumG += colorProp.val[1];
						sumB += colorProp.val[2];
					}

					setElementColor(x, y, z, cv::Vec3b(sumR / numProposals, sumG / numProposals, sumB / numProposals));
				}
			}
		}
	}
}
