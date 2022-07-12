#include <cstdint>
#include <opencv2/opencv.hpp>
#include "open3d/Open3D.h"

#define DO_GRID_VISUALIZATION

class VoxelGrid
{
public:
	VoxelGrid(int sizeX_, int sizeY_, int sizeZ_, double startX_ = 0.0, double startY_ = 0.0, double startZ_ = 0.0, double step_ = 1.0);
	~VoxelGrid();

	int getElement(int x, int y, int z);
	void setElement(int x, int y, int z, uint8_t val);

	void carveSingleImg(cv::Mat img);
	void toPLY();
	void render();

	cv::Point3d voxelToWorld(int x, int y, int z);
	cv::Point2i projectVoxel(int x, int y, int z, cv::Matx44d pose, double imgScale);

	void carve(std::vector<cv::Mat> images, std::vector<cv::Matx44d> poses, 
#ifdef DO_GRID_VISUALIZATION
		std::vector<cv::Mat> results, 
#endif
		double imgScale, float voteTreshold = 1.0);

	int sizeX, sizeY, sizeZ;
	double startX, startY, startZ, step;

private:
	uint8_t ***grid;
};
