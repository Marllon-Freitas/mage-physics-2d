#include "Force.h"
#include "../Core/Body.h"
#include "../Core/PhysicsConstants.h"

#include <algorithm>

Vec2 Force::GenerateDragForce(const Body& body, float k) {
    Vec2 dragForce = Vec2(0, 0);

    if (body.velocity.MagnitudeSquared() > mage::math::EPSILON_SQ) {
        // Calculate the drag direction (inverse of velocity unit vector)
        Vec2 dragDirection = body.velocity.UnitVector() * -1.0;

        // Calculate the drag magnitude, k * |v|^2
        float dragMagnitude = k * body.velocity.MagnitudeSquared();

        // Generate the final drag force with direction and magnitude
        dragForce = dragDirection * dragMagnitude;
    }
    return dragForce;
}

Vec2 Force::GenerateFrictionForce(const Body& body, float k) {
    Vec2 frictionForce = Vec2(0, 0);

    if (body.velocity.MagnitudeSquared() > mage::math::EPSILON_SQ) {
        // Calculate the friction direction (inverse of velocity unit vector)
        Vec2 frictionDirection = body.velocity.UnitVector() * -1.0;

        // Calculate the friction magnitude
        float frictionMagnitude = k;

        // Calculate the final friction force
        frictionForce = frictionDirection * frictionMagnitude;
    }

    return frictionForce;
}

Vec2 Force::GenerateSpringForce(const Body& body, Vec2 anchor, float restLength, float k) {
    Vec2 d = body.position - anchor;
    float distance = d.Magnitude();

    // Prevent division by zero if the object is exactly at the anchor
    if (distance <= mage::math::EPSILON) {
        return Vec2(0.0f, 0.0f);
    }

    Vec2 springDirection = d / distance;

    // Find the spring displacement considering the rest length
    float displacement = distance - restLength;

    // Calculate the magnitude of the spring force
    float springMagnitude = -k * displacement;

    // Calculate the final resulting spring force vector
    return springDirection * springMagnitude;
}

Vec2 Force::GenerateSpringForce(const Body& a, const Body& b, float restLength, float k) {
    Vec2 d = a.position - b.position;
    float distance = d.Magnitude();

    if (distance <= mage::math::EPSILON) {
        return Vec2(0.0f, 0.0f);
    }

    Vec2 springDirection = d / distance;
    float displacement = distance - restLength;
    float springMagnitude = -k * displacement;

    return springDirection * springMagnitude;
}

Vec2 Force::GenerateGravitationalForce(const Body& a, const Body& b, float G, float minDistance, float maxDistance) {
    Vec2 d = (b.position - a.position);
    float distanceSquared = d.MagnitudeSquared();

    // Clamp the distance to prevent astronomical forces at near-zero distances
    // and to cap the force at maximum distances for gameplay reasons.
    distanceSquared = std::clamp(distanceSquared, minDistance, maxDistance);

    // Calculate the direction of the attraction force
    Vec2 attractionDirection = d.UnitVector();

    // Newton's law of universal gravitation: F = G * (m1 * m2) / r^2
    float attractionMagnitude = G * (a.mass * b.mass) / distanceSquared;

    return attractionDirection * attractionMagnitude;
}
