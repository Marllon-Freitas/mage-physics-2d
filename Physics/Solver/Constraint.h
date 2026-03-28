/**
 * @file Constraint.h
 * @brief Base class and implementations for physical constraints (joints, collisions).
 *
 * Constraints limit the relative movement between bodies. This file defines
 * the mathematical framework (Jacobian formulations, warm starting, Baumgarte
 * stabilization) used to solve these limits via Sequential Impulses.
 */

#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "../Math/Mat22.h"
#include "../Math/Mat33.h"
#include "../Core/Body.h"

 /**
  * @brief Abstract base class for all physics constraints.
  */
class Constraint {
public:
    Body* a;
    Body* b;

    Vec2 aPoint; // Constraint anchor in A's local space
    Vec2 bPoint; // Constraint anchor in B's local space

    virtual ~Constraint() = default;

    virtual void PreSolve(float dt) {}
    virtual void Solve() {}
    virtual void PostSolve() {}
};

/**
 * @brief Forces two bodies to share a common anchor point (pivot/hinge).
 */
class JointConstraint : public Constraint {
public:
    JointConstraint();
    JointConstraint(Body* a, Body* b, const Vec2& anchorPoint);

    void PreSolve(float dt) override;
    void Solve() override;
    void PostSolve() override;

private:
    Mat22 m_effectiveMass;
    Vec2  m_cachedLambda;
    Vec2  m_bias;
    Vec2  m_rA;
    Vec2  m_rB;
};

/**
 * @brief Keeps two bodies at a fixed distance from each other.
 */
class DistanceConstraint : public Constraint {
public:
    DistanceConstraint();
    DistanceConstraint(Body* a, Body* b, const Vec2& anchorA, const Vec2& anchorB);

    void PreSolve(float dt) override;
    void Solve() override;
    void PostSolve() override;

private:
    float m_effectiveMass;
    float m_cachedLambda;
    float m_bias;
    float m_length;

    Vec2  m_n;
    Vec2  m_rA;
    Vec2  m_rB;
};

/**
 * @brief Welds two bodies together so they behave as a single rigid body.
 * Restricts translation (X, Y) and rotation.
 */
class WeldConstraint : public Constraint {
public:
    WeldConstraint();
    WeldConstraint(Body* a, Body* b, const Vec2& anchorPoint);

    void PreSolve(float dt) override;
    void Solve() override;
    void PostSolve() override;

private:
    Mat33 m_massMatrix;
    Vec3  m_cachedImpulse;
    Vec3  m_bias;
    float m_referenceAngle;

    Vec2  m_rA;
    Vec2  m_rB;
};

/**
 * @brief Restricts movement to a single axis (piston, elevator, rail).
 */
class PrismaticConstraint : public Constraint {
public:
    PrismaticConstraint();
    PrismaticConstraint(Body* a, Body* b, const Vec2& anchorPoint, const Vec2& axis);

    void PreSolve(float dt) override;
    void Solve() override;
    void PostSolve() override;

private:
    Mat22 m_effectiveMass;
    Vec2  m_cachedLambda;
    Vec2  m_bias;

    Vec2  m_localAxis;
    float m_referenceAngle;

    Vec2  m_perp;
    Vec2  m_rA;
    Vec2  m_rB;
};

/**
 * @brief Resolves body overlap and applies friction at a collision contact point.
 */
class PenetrationConstraint : public Constraint {
    friend class Manifold;

public:
    PenetrationConstraint();
    PenetrationConstraint(Body* a, Body* b,
        const Vec2& aCollisionPoint,
        const Vec2& bCollisionPoint,
        const Vec2& normal);

    void PreSolve(float dt) override;
    void Solve() override;
    void PostSolve() override;

    float GetNormalImpulse() const { return m_normalImpulse; }

private:
    Vec2  m_rA;
    Vec2  m_rB;
    Vec2  m_n;
    Vec2  m_t;

    float m_normalMass;
    float m_tangentMass;

    float m_normalImpulse;
    float m_tangentImpulse;

    float m_bias;
    Vec2  m_localNormal;
    float m_friction;
};

#endif