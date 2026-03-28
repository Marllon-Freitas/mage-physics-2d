/**
 * @file Body.h
 * @brief Defines the rigid body entity, its physical properties, and behaviors.
 *
 * This file contains the core physics object used in the simulation. It handles
 * mass properties, forces, impulses, and maintains the state of the object
 * across physics frames.
 */

#ifndef BODY_H
#define BODY_H

#include "../Collision/Shape.h"
#include "../Math/Vec2.h"

#include <functional>
#include <memory>

// Defines how the physics engine interacts with this body
enum class BodyType {
    Static,     // Infinite mass, unaffected by forces
    Kinematic,  // Infinite mass, moved manually via velocity/code
    Dynamic     // Has mass, reacts to forces, collisions, and gravity
};

// Payload delivered to gameplay code when a collision event occurs
struct CollisionData {
    struct Body* other; // The other body involved in the collision
    Vec2 normal;        // Direction of the collision response
    float depth;        // Penetration depth
    float impactSpeed;  // The relative speed along the normal at impact
};

struct Body {
    // Identity & flags
    void* userData = nullptr;
    BodyType type = BodyType::Dynamic;
    bool isSensor = false;    // Detects collisions but ignores physics response (trigger volumes)
    bool isOneWay = false;    // Passable from below, solid from above
    bool isColliding = false; // General collision state flag

    // Linear motion
    Vec2 position = Vec2(0.0f, 0.0f);
    Vec2 velocity = Vec2(0.0f, 0.0f);
    Vec2 acceleration = Vec2(0.0f, 0.0f);

    // Angular motion
    float rotation = 0.0f;
    float angularVelocity = 0.0f;
    float angularAcceleration = 0.0f;

    // Forces & torque accumulators
    Vec2  sumForces = Vec2(0.0f, 0.0f);
    float sumTorque = 0.0f;

    // Mass properties
    float mass;    // Linear mass
    float invMass; // Cached inverse (0 for Static/Kinematic)
    float I;       // Moment of inertia
    float invI;    // Cached inverse (0 for Static/Kinematic)

    // Material & damping properties
    float restitution = 1.0f;    // Bounciness/elasticity [0.0, 1.0]
    float friction = 0.7f;       // Surface friction      [0.0, 1.0]

    float linearDamping = 0.0f;  // Air resistance/drag (0.0 = vacuum, higher = more drag)
    float angularDamping = 0.1f; // Angular drag (prevents infinite spinning)

    std::unique_ptr<Shape> shape;
    float  boundingRadius = 0.0f;   // Bounding circle radius for broad-phase checks

    // Sleep state
    bool  isSleeping = false;
    float sleepTimer = 0.0f;

    Body(const Shape& shape, float x, float y, float mass);
    Body(const Body&) = delete;
    Body& operator=(const Body&) = delete;

    Body(Body&&) noexcept = default;
    Body& operator=(Body&&) noexcept = default;

    ~Body() = default;

    bool IsStatic()    const { return type == BodyType::Static; }
    bool IsKinematic() const { return type == BodyType::Kinematic; }
    bool IsDynamic()   const { return type == BodyType::Dynamic; }

    void AddForce(const Vec2& force);
    void AddTorque(float torque);
    void ClearForces();
    void ClearTorque();

    void ApplyImpulseLinear(const Vec2& j);
    void ApplyImpulseAngular(float j);
    void ApplyImpulseAtPoint(const Vec2& j, const Vec2& r);

    void IntegrateForces(float dt);
    void IntegrateVelocities(float dt);

    void WakeUp();

    void CalculateBoundingRadius();
    Vec2 LocalSpaceToWorldSpace(const Vec2& point) const;
    Vec2 WorldSpaceToLocalSpace(const Vec2& point) const;
};

#endif