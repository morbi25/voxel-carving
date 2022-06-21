#include "VoxelGrid.h"

VoxelGrid::VoxelGrid(int sizeX_, int sizeY_, int sizeZ_)
{
	sizeX = sizeX_;
	sizeY = sizeY_;
	sizeZ = sizeZ_;

	grid = new uint8_t** [sizeZ];

	for (int z = 0; z < sizeZ; ++z) {
		grid[z] = new uint8_t * [sizeX];
		for (int x = 0; x < sizeX; ++x)
			grid[z][x] = new uint8_t[sizeY];
	}

	for (int z = 0; z < sizeZ_; ++z)
	{
		for (int x = 0; x < sizeX_; ++x)
		{
			for (int y = 0; y < sizeY_; ++y)
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
