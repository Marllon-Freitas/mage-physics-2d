/**
 * @file SpatialHashGrid.h
 * @brief Spatial partitioning system to optimize Broad Phase collision detection.
 */

#ifndef SPATIAL_HASH_GRID_H
#define SPATIAL_HASH_GRID_H

#include "../Core/Body.h"
#include <unordered_map>
#include <vector>

class SpatialHashGrid {
public:
    explicit SpatialHashGrid(float cellSize);
    ~SpatialHashGrid() = default;

    /** @brief Resets all cells for the current frame without reallocating memory. */
    void Clear();

    /** @brief Inserts a body into all grid cells covered by its bounding box. */
    void Insert(Body* body);

    /** @brief Returns unique pairs of bodies potentially colliding. */
    std::vector<std::pair<Body*, Body*>> GetCandidatePairs() const;

    void  SetCellSize(float size) { m_cellSize = size; }
    float GetCellSize()     const { return m_cellSize; }

private:
    float m_cellSize;
    std::unordered_map<int, std::vector<Body*>> m_cells;

    /** @brief Generates a unique 1D hash key from 2D coordinates using prime numbers. */
    int GetHashKey(int x, int y) const;
};

#endif