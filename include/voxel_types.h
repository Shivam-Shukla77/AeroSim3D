#pragma once

#include <raylib.h>
#include <vector>

struct VoxelBlock {
    Vector3 localPosition;
    float mass;
    bool active;
    unsigned char health;
};

struct VoxelGrid {
    Vector3 minBounds;
    Vector3 maxBounds;
    int countX;
    int countY;
    int countZ;
    float totalDryMass;
    Vector3 centerOfMass;
    float inertiaTensor[3][3];

    std::vector<VoxelBlock> blocks;
};

// Voxel Cache Manager
bool GetOrInitializeVoxelGrid(const char* meshFilePath, VoxelGrid& outGrid, int resolutionX, float totalDryMass);

// Offline Voxelizer Pre-processor
bool PrecomputeMeshVoxelization(const char* meshFilePath, const char* outputBinaryPath, int resolutionX, float totalDryMass);

// Runtime Interface
bool LoadVoxelData(const char* binaryFilePath, VoxelGrid& outGrid);
void RecalculateGridProperties(VoxelGrid& grid);
void ScaleVoxelGrid(VoxelGrid& grid, float scaleFactor);