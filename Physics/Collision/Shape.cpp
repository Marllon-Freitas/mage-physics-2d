#include "Shape.h"

#include <algorithm>
#include <limits>
#include <cmath>

CircleShape::CircleShape(float radius) : radius(radius) {}

Shape* CircleShape::Clone() const {
    return new CircleShape(radius);
}

void CircleShape::UpdateVertices(float angle, const Vec2& position) {
    return; // Circles don't have vertices... nothing to do here
}

float CircleShape::GetMomentOfInertia() const {
    // For solid circles, the moment of inertia is 1/2 * r^2
    // But this still needs to be multiplied by the rigidbody's mass
    return 0.5f * (radius * radius);
}

// ============================================================================
// POLYGON SHAPE
// ============================================================================

PolygonShape::PolygonShape(const std::vector<Vec2>& vertices) {
    float minX = std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float maxY = std::numeric_limits<float>::lowest();

    // Initialize the vertices of the polygon shape and set width and height
    for (const auto& vertex : vertices) {
        localVertices.push_back(vertex);
        worldVertices.push_back(vertex);

        // Find min and max X and Y to calculate polygon width and height
        minX = std::min(minX, vertex.x);
        maxX = std::max(maxX, vertex.x);
        minY = std::min(minY, vertex.y);
        maxY = std::max(maxY, vertex.y);
    }
    width = maxX - minX;
    height = maxY - minY;
}

Shape* PolygonShape::Clone() const {
    return new PolygonShape(localVertices);
}

float PolygonShape::PolygonArea() const {
    float area = 0.0f;
    for (int i = 0; i < localVertices.size(); i++) {
        int j = (i + 1) % localVertices.size();
        area += localVertices[i].Cross(localVertices[j]);
    }
    return area / 2.0f;
}

Vec2 PolygonShape::PolygonCentroid() const {
    Vec2 cg(0.0f, 0.0f);
    for (size_t i = 0; i < localVertices.size(); i++) {
        size_t j = (i + 1) % localVertices.size();
        cg += (localVertices[i] + localVertices[j]) * localVertices[i].Cross(localVertices[j]);
    }
    return cg / 6.0f / PolygonArea();
}

float PolygonShape::GetMomentOfInertia() const {
    float acc0 = 0.0f;
    float acc1 = 0.0f;
    const size_t count = localVertices.size();

    for (size_t i = 0; i < count; i++) {
        const auto& a = localVertices[i];
        const auto& b = localVertices[(i + 1) % count];

        const float cross = std::fabs(a.Cross(b));

        acc0 += cross * (a.Dot(a) + b.Dot(b) + a.Dot(b));
        acc1 += cross;
    }

    return (acc1 > 0.0f) ? (acc0 / 6.0f / acc1) : 0.0f;
}

Vec2 PolygonShape::EdgeAt(int index) const {
    int currVertex = index;
    int nextVertex = (index + 1) % worldVertices.size();
    return worldVertices[nextVertex] - worldVertices[currVertex];
}

float PolygonShape::FindMinSeparation(const PolygonShape* other, int& indexReferenceEdge, Vec2& supportPoint) const {
    float separation = std::numeric_limits<float>::lowest();

    // Loop all the vertices of "this" polygon
    for (int i = 0; i < this->worldVertices.size(); i++) {
        Vec2 va = this->worldVertices[i];
        Vec2 normal = this->EdgeAt(i).Normal();

        // Loop all the vertices of the "other" polygon
        float minSep = std::numeric_limits<float>::max();
        Vec2 minVertex;
        for (int j = 0; j < other->worldVertices.size(); j++) {
            Vec2 vb = other->worldVertices[j];
            float proj = (vb - va).Dot(normal);
            if (proj < minSep) {
                minSep = proj;
                minVertex = vb;
            }
        }
        if (minSep > separation) {
            separation = minSep;
            indexReferenceEdge = i;
            supportPoint = minVertex;
        }
    }
    return separation;
}

int PolygonShape::FindIncidentEdge(const Vec2& normal) const {
    int indexIncidentEdge = 0;
    float minProj = std::numeric_limits<float>::max();
    for (size_t i = 0; i < this->worldVertices.size(); ++i) {
        auto edgeNormal = this->EdgeAt(i).Normal();
        auto proj = edgeNormal.Dot(normal);
        if (proj < minProj) {
            minProj = proj;
            indexIncidentEdge = i;
        }
    }
    return indexIncidentEdge;
}

int PolygonShape::ClipSegmentToLine(const std::vector<Vec2>& contactsIn, std::vector<Vec2>& contactsOut, const Vec2& c0, const Vec2& c1) const {
    // Start with no output points
    int numOut = 0;

    // Calculate the distance of end points to the line
    Vec2 normal = (c1 - c0).Normalize();
    float dist0 = (contactsIn[0] - c0).Cross(normal);
    float dist1 = (contactsIn[1] - c0).Cross(normal);

    // If the points are behind the plane
    if (dist0 <= 0.0f)
        contactsOut[numOut++] = contactsIn[0];
    if (dist1 <= 0.0f)
        contactsOut[numOut++] = contactsIn[1];

    // If the points are on different sides of the plane (one distance is negative and the other is positive)
    if (dist0 * dist1 < 0) {
        float totalDist = dist0 - dist1;

        // Fint the intersection using linear interpolation: lerp(start,end) => start + t*(end-start)
        float t = dist0 / (totalDist);
        Vec2 contact = contactsIn[0] + (contactsIn[1] - contactsIn[0]) * t;
        contactsOut[numOut] = contact;
        numOut++;
    }
    return numOut;
}

void PolygonShape::UpdateVertices(float angle, const Vec2& position) {
    // Loop all the vertices, transforming from local to world space
    for (size_t i = 0; i < localVertices.size(); i++) {
        // First rotate, then we translate
        worldVertices[i] = localVertices[i].Rotate(angle);
        worldVertices[i] += position;
    }
}

// ============================================================================
// BOX SHAPE
// ============================================================================

BoxShape::BoxShape(float width, float height) {
    // width and height are inherited from PolygonShape base class
    this->width = width;
    this->height = height;

    // Using initializer lists for immediate sequential memory allocation
    localVertices = {
        Vec2(-width / 2.0f, -height / 2.0f),
        Vec2(+width / 2.0f, -height / 2.0f),
        Vec2(+width / 2.0f, +height / 2.0f),
        Vec2(-width / 2.0f, +height / 2.0f)
    };

    worldVertices = localVertices;
}

Shape* BoxShape::Clone() const {
    return new BoxShape(width, height);
}

float BoxShape::GetMomentOfInertia() const {
    // For a rectangle, the moment of inertia is 1/12 * (w^2 + h^2)
    // But this still needs to be multiplied by the rigidbody's mass
    return (1.0f / 12.0f) * (width * width + height * height);
}

// ============================================================================
// CAPSULE SHAPE
// ============================================================================

CapsuleShape::CapsuleShape(float radius, float length) {
    this->radius = radius;
    this->length = length;

    // Creates the spine aligned vertically along the Y axis by default
    float halfLength = length / 2.0f;
    localP1 = Vec2(0.0f, -halfLength);
    localP2 = Vec2(0.0f, halfLength);

    worldP1 = localP1;
    worldP2 = localP2;
}

Shape* CapsuleShape::Clone() const {
    return new CapsuleShape(radius, length);
}

void CapsuleShape::UpdateVertices(float angle, const Vec2& position) {
    // Transform the spine from local to world space based on the body's rotation
    worldP1 = localP1.Rotate(angle) + position;
    worldP2 = localP2.Rotate(angle) + position;
}

float CapsuleShape::GetMomentOfInertia() const {
    // Approximation: Treat the capsule as a bounding box for inertia purposes.
    // A true capsule MoI is complex, but the box approximation is standard for games.
    float width = radius * 2.0f;
    float totalHeight = length + (radius * 2.0f);
    return (1.0f / 12.0f) * (width * width + totalHeight * totalHeight);
}