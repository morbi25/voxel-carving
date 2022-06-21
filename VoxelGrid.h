#include <cstdint>

class VoxelGrid
{
public:

	VoxelGrid(int sizeX_, int sizeY_, int sizeZ_);
	~VoxelGrid();

	int getElement(int x, int y, int z);
	void setElement(int x, int y, int z, uint8_t val);

	
	int sizeX, sizeY, sizeZ;

private:
	uint8_t*** grid;
};
