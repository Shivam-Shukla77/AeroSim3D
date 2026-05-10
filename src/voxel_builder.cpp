#include "voxel_types.h"
#include <fstream>
#include <cmath>
#include <raymath.h>

bool PrecomputeMeshVoxelization(const char* meshFilePath, const char* outputBinaryPath, int resolutionX, float totalDryMass) {
    Model model = LoadModel(meshFilePath);
    if (model.meshCount == 0 || model.meshes == nullptr) {
        TraceLog(LOG_ERROR, "CRITICAL: Failed to load model for voxelization [%s]", meshFilePath);
        UnloadModel(model);
        return false;
    }

    BoundingBox bbox = GetModelBoundingBox(model);
    Vector3 size = Vector3Subtract(bbox.max, bbox.min);

    float voxelSize = size.x / static_cast<float>(resolutionX);
    int countX = resolutionX;
    int countY = static_cast<int>(std::ceil(size.y / voxelSize));
    int countZ = static_cast<int>(std::ceil(size.z / voxelSize));

    VoxelGrid grid;
    grid.minBounds = bbox.min;
    grid.maxBounds = bbox.max;
    grid.countX = countX;
    grid.countY = countY;
    grid.countZ = countZ;
    grid.totalDryMass = totalDryMass;
    
    size_t totalVoxels = static_cast<size_t>(countX) * static_cast<size_t>(countY) * static_cast<size_t>(countZ);
    grid.blocks.resize(totalVoxels);

    for (int z = 0; z < countZ; ++z) {
        for (int y = 0; y < countY; ++y) {
            for (int x = 0; x < countX; ++x) {
                size_t index = static_cast<size_t>(x) + 
                               static_cast<size_t>(y) * static_cast<size_t>(countX) + 
                               static_cast<size_t>(z) * static_cast<size_t>(countX) * static_cast<size_t>(countY);
                grid.blocks[index].localPosition = {
                    bbox.min.x + (x + 0.5f) * voxelSize,
                    bbox.min.y + (y + 0.5f) * voxelSize,
                    bbox.min.z + (z + 0.5f) * voxelSize
                };
                grid.blocks[index].mass = 0.0f;
                grid.blocks[index].active = false;
                grid.blocks[index].health = 100;
            }
        }
    }

    int activeCount = 0;
    for (int m = 0; m < model.meshCount; m++) {
        Mesh mesh = model.meshes[m];
        if (mesh.vertexCount < 3 || mesh.vertices == nullptr) {
            TraceLog(LOG_ERROR, "CRITICAL: Invalid geometry in mesh %d of [%s]", m, meshFilePath);
            UnloadModel(model);
            return false;
        }
        float* vertices = mesh.vertices;
        unsigned short* indices = mesh.indices;
        
        int triangleCount = mesh.triangleCount;
        for (int t = 0; t < triangleCount; ++t) {
            Vector3 v0, v1, v2;
            if (indices) {
                v0 = {vertices[indices[t * 3 + 0] * 3], vertices[indices[t * 3 + 0] * 3 + 1], vertices[indices[t * 3 + 0] * 3 + 2]};
                v1 = {vertices[indices[t * 3 + 1] * 3], vertices[indices[t * 3 + 1] * 3 + 1], vertices[indices[t * 3 + 1] * 3 + 2]};
                v2 = {vertices[indices[t * 3 + 2] * 3], vertices[indices[t * 3 + 2] * 3 + 1], vertices[indices[t * 3 + 2] * 3 + 2]};
            } else {
                v0 = {vertices[t * 9], vertices[t * 9 + 1], vertices[t * 9 + 2]};
                v1 = {vertices[t * 9 + 3], vertices[t * 9 + 4], vertices[t * 9 + 5]};
                v2 = {vertices[t * 9 + 6], vertices[t * 9 + 7], vertices[t * 9 + 8]};
            }

            // Dense barycentric point sampling along the triangle face to ensure voxel overlap
            const int samples = 10;
            for (int i = 0; i <= samples; ++i) {
                for (int j = 0; j <= samples - i; ++j) {
                    float u = static_cast<float>(i) / samples;
                    float v = static_cast<float>(j) / samples;
                    float w = 1.0f - u - v;

                    Vector3 p = {
                        u * v0.x + v * v1.x + w * v2.x,
                        u * v0.y + v * v1.y + w * v2.y,
                        u * v0.z + v * v1.z + w * v2.z
                    };

                    int vx = static_cast<int>((p.x - bbox.min.x) / voxelSize);
                    int vy = static_cast<int>((p.y - bbox.min.y) / voxelSize);
                    int vz = static_cast<int>((p.z - bbox.min.z) / voxelSize);

                    if (vx >= 0 && vx < countX && vy >= 0 && vy < countY && vz >= 0 && vz < countZ) {
                        size_t index = static_cast<size_t>(vx) + 
                                       static_cast<size_t>(vy) * static_cast<size_t>(countX) + 
                                       static_cast<size_t>(vz) * static_cast<size_t>(countX) * static_cast<size_t>(countY);
                        if (!grid.blocks[index].active) {
                            grid.blocks[index].active = true;
                            activeCount++;
                        }
                    }
                }
            }
        }
    }

    UnloadModel(model);
    if (activeCount == 0) return false;

    float massPerVoxel = totalDryMass / static_cast<float>(activeCount);
    for (size_t i = 0; i < totalVoxels; ++i) {
        if (grid.blocks[i].active) {
            grid.blocks[i].mass = massPerVoxel;
        }
    }

    RecalculateGridProperties(grid);

    std::ofstream out(outputBinaryPath, std::ios::binary);
    if (!out) return false;

    // Serialize metadata header sequentially to avoid struct padding mismatches
    auto writeVector3 = [&out](const Vector3& v) {
        out.write(reinterpret_cast<const char*>(&v.x), sizeof(float));
        out.write(reinterpret_cast<const char*>(&v.y), sizeof(float));
        out.write(reinterpret_cast<const char*>(&v.z), sizeof(float));
    };
    writeVector3(grid.minBounds);
    writeVector3(grid.maxBounds);
    out.write(reinterpret_cast<const char*>(&grid.countX), sizeof(int));
    out.write(reinterpret_cast<const char*>(&grid.countY), sizeof(int));
    out.write(reinterpret_cast<const char*>(&grid.countZ), sizeof(int));
    out.write(reinterpret_cast<const char*>(&grid.totalDryMass), sizeof(float));
    writeVector3(grid.centerOfMass);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out.write(reinterpret_cast<const char*>(&grid.inertiaTensor[i][j]), sizeof(float));
        }
    }
    
    // Then dump the full dense array block
    out.write(reinterpret_cast<const char*>(grid.blocks.data()), grid.blocks.size() * sizeof(VoxelBlock));
    out.close();

    return true;
}