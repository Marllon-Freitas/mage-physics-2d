/**
 * @file Shape.h
 * @brief Geometric shapes used for collision detection and rigid body properties.
 */

#ifndef SHAPE_H
#define SHAPE_H

#include "../Math/Vec2.h"
#include <vector>

enum ShapeType {
    CIRCLE,
    POLYGON,
    BOX,
    CAPSULE
};

/**
 * @struct Shape
 * @brief Abstract base class for all physical shapes.
 */
struct Shape {
    virtual ~Shape() = default;

    virtual ShapeType GetType() const = 0;
    virtual Shape* Clone() const = 0;

    virtual void UpdateVertices(float angle, const Vec2& position) = 0;
    virtual float GetMomentOfInertia() const = 0;
};

struct CircleShape : public Shape {
    float radius;

    explicit CircleShape(float radius);
    virtual ~CircleShape() = default;

    ShapeType GetType() const override { return CIRCLE; }
    Shape* Clone() const override;
    void UpdateVertices(float angle, const Vec2& position) override;
    float GetMomentOfInertia() const override;
};

struct PolygonShape : public Shape {
    float width = 0.0f;
    float height = 0.0f;

    std::vector<Vec2> localVertices;
    std::vector<Vec2> worldVertices;

    PolygonShape() = default;
    explicit PolygonShape(const std::vector<Vec2>& vertices);
    virtual ~PolygonShape() = default;

    ShapeType GetType() const override { return POLYGON; }
    Shape* Clone() const override;

    void UpdateVertices(float angle, const Vec2& position) override;
    float GetMomentOfInertia() const override;

    float PolygonArea() const;
    Vec2 PolygonCentroid() const;
    Vec2 EdgeAt(int index) const;

    float FindMinSeparation(const PolygonShape* other, int& indexReferenceEdge, Vec2& supportPoint) const;
    int FindIncidentEdge(const Vec2& normal) const;
    int ClipSegmentToLine(const std::vector<Vec2>& contactsIn, std::vector<Vec2>& contactsOut, const Vec2& c0, const Vec2& c1) const;
};

struct BoxShape : public PolygonShape {
    BoxShape(float width, float height);
    virtual ~BoxShape() = default;

    ShapeType GetType() const override { return BOX; }
    Shape* Clone() const override;
    float GetMomentOfInertia() const override;
};

struct CapsuleShape : public Shape {
    float radius;
    float length; // Distance between the centers of the two end-circles

    Vec2 localP1, localP2; // Local spine
    Vec2 worldP1, worldP2; // World spine

    CapsuleShape(float radius, float length);
    virtual ~CapsuleShape() = default;

    ShapeType GetType() const override { return CAPSULE; }
    Shape* Clone() const override;
    void UpdateVertices(float angle, const Vec2& position) override;
    float GetMomentOfInertia() const override;
};

#endif