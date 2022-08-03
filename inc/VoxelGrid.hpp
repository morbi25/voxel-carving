#pragma once

#include <cstdint>
#include <opencv2/opencv.hpp>
#include "open3d/Open3D.h"

//#define DO_GRID_VISUALIZATION
#define USE_PARALLELIZATION

class VoxelGrid
{
	uint8_t ***grid;
	cv::Vec3b ***colorGrid;

	/**
	 * @brief Convert voxel grid to ply file
	 *
	 */
	void toPLY();

	/**
	 * @brief Define voxel with respect to the world coordinate system using the pre-defined origin and the step size
	 *
	 * @param x coordinate
	 * @param y coordinate
	 * @param z coordinate
	 * @return Voxel coordinates with respect to world coordinate system's origin and step size
	 */
	cv::Point3d voxelToWorld(int x, int y, int z);

	/**
	 * @brief Perspective projection of the 3D voxel to 2D pixel coordinates
	 *
	 * @param x coordinate
	 * @param y coordinate
	 * @param z coordinate
	 * @param pose Rigid body transformation that transforms points from the board coordinate system to the camera coordinate system
	 * @param imgScale Image scaling factor to adapt camera intrinsics accordingly
	 * @param zc Depth value of the voxel used in weights of coloring algorithm
	 * @return Projected 2D pixel coordinates of the voxel
	 */
	cv::Point2i projectVoxel(int x, int y, int z, cv::Matx44d pose, double imgScale, double& zc);

public:
	int sizeX, sizeY, sizeZ;
	double startX, startY, startZ, step;

	/**
	 * @brief Construct a new Voxel Grid object
	 *
	 * @param sizeX_ Size of voxel grid in x direction
	 * @param sizeY_ Size of voxel grid in y direction
	 * @param sizeZ_ Size of voxel grid in z direction
	 * @param startX_ Origin x coordinate of voxel grid
	 * @param startY_ Origin y coordinate of voxel grid
	 * @param startZ_ Origin z coordinate of voxel grid
	 * @param step_ Step size
	 */
	VoxelGrid(int sizeX_, int sizeY_, int sizeZ_, double startX_ = 0.0, double startY_ = 0.0, double startZ_ = 0.0, double step_ = 1.0);

	/**
	 * @brief Destroy the Voxel Grid object
	 *
	 */
	~VoxelGrid();

	/**
	 * @brief Get grid element
	 *
	 * @param x coordinate
	 * @param y coordinate
	 * @param z coordinate
	 * @return Grid value indicating if the voxel is kept or not
	 */
	int getElement(int x, int y, int z);

	/**
	 * @brief Set grid element
	 *
	 * @param x coordinate
	 * @param y coordinate
	 * @param z coordinate
	 * @param val Value to which the voxel grid element should be set
	 */
	void setElement(int x, int y, int z, uint8_t val);

	/**
	 * @brief Get grid element color
	 *
	 * @param x coordinate
	 * @param y coordinate
	 * @param z coordinate
	 * @return RGB color value of the voxel grid element
	 */
	cv::Vec3b getElementColor(int x, int y, int z);

	/**
	 * @brief Get grid element color
	 *
	 * @param x coordinate
	 * @param y coordinate
	 * @param z coordinate
	 * @param rgbColor RGB color value to which the voxel grid element color should be set
	 */
	void setElementColor(int x, int y, int z, cv::Vec3b rgbColor);

	/**
	 * @brief Render the voxel grid with Open3D
	 *
	 */
	void render();

	/**
	 * @brief Perform voxel carving
	 *
	 * @param imageMetas Pre-processed images and meta data
	 * @param imgScale Image scaling factor for perspective projection
	 * @param voteTreshold Necessary percentage of image votes to carve the voxel
	 */
	void carve(std::vector<ImageMeta> imageMetas, double imgScale, float voteTreshold = 1.0);
};
