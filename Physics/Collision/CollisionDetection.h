/**
 * @file CollisionDetection.h
 * @brief Core algorithms for detecting intersections and generating contact manifolds.
 *
 * Implements the Separating Axis Theorem (SAT) for polygon collisions and
 * mathematical intersection tests for circles, capsules, and raycasts.
 */

#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include "../Core/Body.h"
#include "Contact.h"
#include <vector>

 /**
  * @brief Payload containing the result of a raycast query against a rigid body.
  */
struct RaycastResult {
    bool  hit = false;
    Body* body = nullptr;
    Vec2  point;
    Vec2  normal;
    float t = 1.0f; // Time of impact (0.0 to 1.0 along the ray)
};

/**
 * @brief Static utility class handling all Narrow Phase collision tests.
 */
struct CollisionDetection {
    // Dispatcher
    static bool IsColliding(Body* a, Body* b, std::vector<Contact>& contacts);

    // Primitive Tests
    static bool IsCollidingCircleCircle(Body* a, Body* b, std::vector<Contact>& contacts);
    static bool IsCollidingCapsuleCircle(Body* capsule, Body* circle, std::vector<Contact>& contacts);
    static bool IsCollidingCapsuleCapsule(Body* a, Body* b, std::vector<Contact>& contacts);
    static bool IsCollidingCapsulePolygon(Body* capsule, Body* poly, std::vector<Contact>& contacts);
    static bool IsCollidingPolygonPolygon(Body* a, Body* b, std::vector<Contact>& contacts);
    static bool IsCollidingPolygonCircle(Body* polygon, Body* circle, std::vector<Contact>& contacts);

    // Raycast Queries
    static bool RaycastCircle(Body* circleBody, const Vec2& start, const Vec2& end, RaycastResult& out);
    static bool RaycastPolygon(Body* polygonBody, const Vec2& start, const Vec2& end, RaycastResult& out);
    static bool RaycastCapsule(Body* capsuleBody, const Vec2& start, const Vec2& end, RaycastResult& out);
};

#endif