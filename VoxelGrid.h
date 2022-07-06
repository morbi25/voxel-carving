#include <cstdint>
#include <opencv2/opencv.hpp>

class VoxelGrid
{
public:
	VoxelGrid::VoxelGrid(int sizeX_, int sizeY_, int sizeZ_, float startX_ = 0.0, float startY_ = 0.0, float startZ_ = 0.0, float step_ = 1.0);
	~VoxelGrid();

	int getElement(int x, int y, int z);
	void setElement(int x, int y, int z, uint8_t val);

	void carveSingleImg(cv::Mat img);
	void toPLY();
	void render();

	cv::Point3f VoxelGrid::voxelToWorld(int x, int y, int z);
	cv::Point2i projectVoxel(int x, int y, int z, cv::Mat P);
	void carve(std::vector<cv::Mat> images, std::vector<cv::Mat> PMatrices, float voteTreshold = 1.0);
	
	int sizeX, sizeY, sizeZ;
	float startX, startY, startZ, step;

private:
	uint8_t ***grid;
};
