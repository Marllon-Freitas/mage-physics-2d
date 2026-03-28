#include "SpatialHashGrid.h"
#include <algorithm>
#include <cmath>

SpatialHashGrid::SpatialHashGrid(float cellSize) : m_cellSize(cellSize) {
    m_cells.reserve(1024);
}

int SpatialHashGrid::GetHashKey(int x, int y) const {
    return x * 1000003 + y;
}

void SpatialHashGrid::Clear() {
    for (auto& [key, bodyList] : m_cells) {
        bodyList.clear();
    }
}

void SpatialHashGrid::Insert(Body* body) {
    const float minX = body->position.x - body->boundingRadius;
    const float maxX = body->position.x + body->boundingRadius;
    const float minY = body->position.y - body->boundingRadius;
    const float maxY = body->position.y + body->boundingRadius;

    const int cellMinX = static_cast<int>(std::floor(minX / m_cellSize));
    const int cellMaxX = static_cast<int>(std::floor(maxX / m_cellSize));
    const int cellMinY = static_cast<int>(std::floor(minY / m_cellSize));
    const int cellMaxY = static_cast<int>(std::floor(maxY / m_cellSize));

    for (int x = cellMinX; x <= cellMaxX; ++x) {
        for (int y = cellMinY; y <= cellMaxY; ++y) {
            m_cells[GetHashKey(x, y)].push_back(body);
        }
    }
}

std::vector<std::pair<Body*, Body*>> SpatialHashGrid::GetCandidatePairs() const {
    std::vector<std::pair<Body*, Body*>> candidates;
    candidates.reserve(m_cells.size() * 2);

    for (const auto& [key, cellBodies] : m_cells) {
        const size_t count = cellBodies.size();
        if (count < 2) continue;

        for (size_t i = 0; i < count; ++i) {
            for (size_t j = i + 1; j < count; ++j) {
                Body* a = cellBodies[i];
                Body* b = cellBodies[j];

                if (a->IsStatic() && b->IsStatic()) continue;
                if (a->isSleeping && b->isSleeping) continue;
                if ((a->isSleeping && b->IsStatic()) || (a->IsStatic() && b->isSleeping)) continue;

                if (a > b) std::swap(a, b);

                const float radiusSum = a->boundingRadius + b->boundingRadius;
                const Vec2 diff = b->position - a->position;

                if (diff.MagnitudeSquared() <= (radiusSum * radiusSum)) {
                    candidates.emplace_back(a, b);
                }
            }
        }
    }

    std::sort(candidates.begin(), candidates.end());
    candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

    return candidates;
}