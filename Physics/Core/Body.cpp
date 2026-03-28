#include "Body.h"
#include "../Collision/Shape.h"
#include "PhysicsConstants.h"

#include <algorithm>
#include <cmath>

Body::Body(const Shape& shape, float x, float y, float mass) {
    this->shape = std::unique_ptr<Shape>(shape.Clone());

    position = Vec2(x, y);
    velocity = Vec2(0.0f, 0.0f);
    acceleration = Vec2(0.0f, 0.0f);

    rotation = 0.0f;
    angularVelocity = 0.0f;
    angularAcceleration = 0.0f;

    sumForces = Vec2(0.0f, 0.0f);
    sumTorque = 0.0f;

    restitution = 1.0f;
    friction = 0.7f;

    this->mass = mass;

    // Bodies with zero mass are immovable (Static)
    if (mass > mage::math::EPSILON) {
        invMass = 1.0f / mass;
        type = BodyType::Dynamic;
    }
    else {
        invMass = 0.0f;
        type = BodyType::Static;
    }

    I = shape.GetMomentOfInertia() * mass;
    if (I > mage::math::EPSILON) {
        invI = 1.0f / I;
    }
    else {
        invI = 0.0f;
    }

    this->shape->UpdateVertices(rotation, position);
    CalculateBoundingRadius();
}

void Body::CalculateBoundingRadius() {
    switch (shape->GetType()) {
    case CIRCLE: {
        auto* circle = static_cast<CircleShape*>(shape.get());
        boundingRadius = circle->radius;
        break;
    }
    case BOX: {
        auto* box = static_cast<BoxShape*>(shape.get());
        boundingRadius = std::sqrtf(box->width * box->width + box->height * box->height) * 0.5f;
        break;
    }
    case POLYGON: {
        auto* polygon = static_cast<PolygonShape*>(shape.get());
        float maxDistSq = 0.0f;
        for (auto& v : polygon->localVertices) {
            float distSq = v.MagnitudeSquared();
            if (distSq > maxDistSq) maxDistSq = distSq;
        }
        boundingRadius = std::sqrtf(maxDistSq);
        break;
    }
    case CAPSULE: {
        auto* capsule = static_cast<CapsuleShape*>(shape.get());
        boundingRadius = (capsule->length * 0.5f) + capsule->radius;
        break;
    }
    default:
        boundingRadius = 0.0f;
        break;
    }
}

void Body::AddForce(const Vec2& force) {
    sumForces += force;
}

void Body::AddTorque(float torque) {
    sumTorque += torque;
}

void Body::ClearForces() {
    sumForces = Vec2(0.0f, 0.0f);
}

void Body::ClearTorque() {
    sumTorque = 0.0f;
}

void Body::ApplyImpulseLinear(const Vec2& j) {
    if (IsStatic() || IsKinematic()) return;
    if (j.MagnitudeSquared() > mage::physics::SLEEP_IMPULSE_THRESHOLD * mage::physics::SLEEP_IMPULSE_THRESHOLD)
        WakeUp();
    velocity += j * invMass;
}

void Body::ApplyImpulseAngular(float j) {
    if (IsStatic() || IsKinematic()) return;
    if (j * j > mage::physics::SLEEP_IMPULSE_THRESHOLD * mage::physics::SLEEP_IMPULSE_THRESHOLD)
        WakeUp();
    angularVelocity += j * invI;
}

void Body::ApplyImpulseAtPoint(const Vec2& j, const Vec2& r) {
    if (IsStatic() || IsKinematic()) return;
    velocity += j * invMass;
    angularVelocity += r.Cross(j) * invI;
}

void Body::IntegrateForces(float dt) {
    if (IsStatic() || IsKinematic() || isSleeping) return;

    acceleration = sumForces * invMass;
    velocity += acceleration * dt;

    velocity *= 1.0f / (1.0f + linearDamping * dt);
    angularVelocity *= 1.0f / (1.0f + angularDamping * dt);

    ClearForces();
    ClearTorque();
}

void Body::IntegrateVelocities(float dt) {
    if (IsStatic() || isSleeping) return;

    position += velocity * dt;
    rotation += angularVelocity * dt;

    shape->UpdateVertices(rotation, position);
}

void Body::WakeUp() {
    if (!IsDynamic()) return;

    if (isSleeping) {
        isSleeping = false;
        sleepTimer = 0.0f;
    }
}

Vec2 Body::LocalSpaceToWorldSpace(const Vec2& point) const {
    return point.Rotate(rotation) + position;
}

Vec2 Body::WorldSpaceToLocalSpace(const Vec2& point) const {
    float tx = point.x - position.x;
    float ty = point.y - position.y;
    float cos = std::cos(-rotation);
    float sin = std::sin(-rotation);

    return Vec2(
        cos * tx - sin * ty,
        cos * ty + sin * tx
    );
}