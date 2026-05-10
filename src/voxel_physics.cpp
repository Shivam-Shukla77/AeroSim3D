#include "voxel_types.h"
#include <fstream>
#include <raymath.h>
#include <filesystem>

bool GetOrInitializeVoxelGrid(const char* meshFilePath, VoxelGrid& outGrid, int resolutionX, float totalDryMass) {
    std::filesystem::path cachePath = meshFilePath;
    cachePath.replace_extension(".voxdata");
    std::string cacheFilePath = cachePath.string();

    if (std::filesystem::exists(cachePath)) {
        TraceLog(LOG_INFO, "VOXEL CACHE: Hit found for [%s]. Loading binary data directly...", cacheFilePath.c_str());
        return LoadVoxelData(cacheFilePath.c_str(), outGrid);
    } else {
        TraceLog(LOG_INFO, "VOXEL CACHE: Miss for [%s]. Precomputing new voxel grid offline...", meshFilePath);
        if (!PrecomputeMeshVoxelization(meshFilePath, cacheFilePath.c_str(), resolutionX, totalDryMass)) {
            TraceLog(LOG_ERROR, "CRITICAL: Offline voxelization failed for [%s]", meshFilePath);
            return false;
        }
        TraceLog(LOG_INFO, "VOXEL CACHE: Serialization complete. Loading newly generated grid...");
        return LoadVoxelData(cacheFilePath.c_str(), outGrid);
    }
}

bool LoadVoxelData(const char* binaryFilePath, VoxelGrid& outGrid) {
    std::ifstream in(binaryFilePath, std::ios::binary);
    if (!in) return false;

    // Stream header metadata sequentially
    auto readVector3 = [&in](Vector3& v) {
        in.read(reinterpret_cast<char*>(&v.x), sizeof(float));
        in.read(reinterpret_cast<char*>(&v.y), sizeof(float));
        in.read(reinterpret_cast<char*>(&v.z), sizeof(float));
    };
    
    readVector3(outGrid.minBounds);
    readVector3(outGrid.maxBounds);
    in.read(reinterpret_cast<char*>(&outGrid.countX), sizeof(int));
    in.read(reinterpret_cast<char*>(&outGrid.countY), sizeof(int));
    in.read(reinterpret_cast<char*>(&outGrid.countZ), sizeof(int));
    in.read(reinterpret_cast<char*>(&outGrid.totalDryMass), sizeof(float));
    readVector3(outGrid.centerOfMass);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            in.read(reinterpret_cast<char*>(&outGrid.inertiaTensor[i][j]), sizeof(float));
        }
    }

    // OOM & Corruption Protection: Validate dimensions safely
    if (outGrid.countX <= 0 || outGrid.countY <= 0 || outGrid.countZ <= 0) {
        TraceLog(LOG_ERROR, "LoadVoxelData: Invalid grid dimensions (%d, %d, %d)", outGrid.countX, outGrid.countY, outGrid.countZ);
        in.close();
        return false;
    }

    // Evaluate memory allocation bounds (limit total array to roughly ~256MB)
    size_t totalVoxels = static_cast<size_t>(outGrid.countX) * static_cast<size_t>(outGrid.countY) * static_cast<size_t>(outGrid.countZ);
    const size_t MAX_VOXELS = 16777216; // 256 ^ 3 cells limit
    
    if (totalVoxels > MAX_VOXELS) {
        TraceLog(LOG_ERROR, "LoadVoxelData: Voxel count (%zu) exceeds max limit (%zu)", totalVoxels, MAX_VOXELS);
        in.close();
        return false;
    }

    // Pre-allocate after verification and stream raw grid blocks
    outGrid.blocks.resize(totalVoxels);
    in.read(reinterpret_cast<char*>(outGrid.blocks.data()), totalVoxels * sizeof(VoxelBlock));
    
    in.close();
    return true;
}

void RecalculateGridProperties(VoxelGrid& grid) {
    float newTotalMass = 0.0f;
    Vector3 newCoM = {0.0f, 0.0f, 0.0f};

    for (const auto& block : grid.blocks) {
        if (block.active) {
            newTotalMass += block.mass;
            newCoM = Vector3Add(newCoM, Vector3Scale(block.localPosition, block.mass));
        }
    }

    if (newTotalMass > 0.0f) {
        newCoM = Vector3Scale(newCoM, 1.0f / newTotalMass);
    }
    grid.totalDryMass = newTotalMass;
    grid.centerOfMass = newCoM;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            grid.inertiaTensor[i][j] = 0.0f;
        }
    }

    for (const auto& block : grid.blocks) {
        if (block.active) {
            Vector3 r = Vector3Subtract(block.localPosition, grid.centerOfMass);
            float r2 = Vector3DotProduct(r, r);
            grid.inertiaTensor[0][0] += block.mass * (r2 - r.x * r.x);
            grid.inertiaTensor[1][1] += block.mass * (r2 - r.y * r.y);
            grid.inertiaTensor[2][2] += block.mass * (r2 - r.z * r.z);
            grid.inertiaTensor[0][1] -= block.mass * r.x * r.y;
            grid.inertiaTensor[0][2] -= block.mass * r.x * r.z;
            grid.inertiaTensor[1][2] -= block.mass * r.y * r.z;
        }
    }
    grid.inertiaTensor[1][0] = grid.inertiaTensor[0][1];
    grid.inertiaTensor[2][0] = grid.inertiaTensor[0][2];
    grid.inertiaTensor[2][1] = grid.inertiaTensor[1][2];
}