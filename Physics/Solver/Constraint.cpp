#include "Constraint.h"
#include "../Core/PhysicsConstants.h"
#include "../Core/Body.h"

#include <algorithm>

JointConstraint::JointConstraint()
    : Constraint(), m_cachedLambda(0.0f, 0.0f), m_bias(0.0f, 0.0f) {
}

JointConstraint::JointConstraint(Body* a, Body* b, const Vec2& anchorPoint)
    : Constraint(), m_cachedLambda(0.0f, 0.0f), m_bias(0.0f, 0.0f) {
    this->a = a;
    this->b = b;
    aPoint = a->WorldSpaceToLocalSpace(anchorPoint);
    bPoint = b->WorldSpaceToLocalSpace(anchorPoint);
}

void JointConstraint::PreSolve(float dt) {
    const Vec2 pa = a->LocalSpaceToWorldSpace(aPoint);
    const Vec2 pb = b->LocalSpaceToWorldSpace(bPoint);
    m_rA = pa - a->position;
    m_rB = pb - b->position;

    float invMassSum = a->invMass + b->invMass;

    float k00 = invMassSum + (m_rA.y * m_rA.y) * a->invI + (m_rB.y * m_rB.y) * b->invI;
    float k01 = -m_rA.y * m_rA.x * a->invI - m_rB.y * m_rB.x * b->invI;
    float k10 = k01;
    float k11 = invMassSum + (m_rA.x * m_rA.x) * a->invI + (m_rB.x * m_rB.x) * b->invI;

    Mat22 K(k00, k01, k10, k11);
    m_effectiveMass = K.Inverse();

    const float beta = 0.05f;
    const float slop = 0.5f;
    Vec2 C = pb - pa;

    m_bias.x = (beta / dt) * (std::abs(C.x) > slop ? C.x : 0.0f);
    m_bias.y = (beta / dt) * (std::abs(C.y) > slop ? C.y : 0.0f);

    a->ApplyImpulseLinear(m_cachedLambda * -1.0f);
    a->ApplyImpulseAngular(-m_rA.Cross(m_cachedLambda));

    b->ApplyImpulseLinear(m_cachedLambda);
    b->ApplyImpulseAngular(m_rB.Cross(m_cachedLambda));
}

void JointConstraint::Solve() {
    Vec2 vaPoint = a->velocity + Vec2(-a->angularVelocity * m_rA.y, a->angularVelocity * m_rA.x);
    Vec2 vbPoint = b->velocity + Vec2(-b->angularVelocity * m_rB.y, b->angularVelocity * m_rB.x);

    Vec2 jv = vbPoint - vaPoint;

    Vec2 lambda = m_effectiveMass * ((jv + m_bias) * -1.0f);
    m_cachedLambda += lambda;

    a->ApplyImpulseLinear(lambda * -1.0f);
    a->ApplyImpulseAngular(-m_rA.Cross(lambda));

    b->ApplyImpulseLinear(lambda);
    b->ApplyImpulseAngular(m_rB.Cross(lambda));
}

void JointConstraint::PostSolve() {}

// Distance Constraint (Rod / Rope / Spring)
DistanceConstraint::DistanceConstraint()
    : Constraint(), m_effectiveMass(0.0f), m_cachedLambda(0.0f), m_bias(0.0f), m_length(0.0f) {
}

DistanceConstraint::DistanceConstraint(Body* a, Body* b, const Vec2& anchorA, const Vec2& anchorB)
    : Constraint(), m_effectiveMass(0.0f), m_cachedLambda(0.0f), m_bias(0.0f) {
    this->a = a;
    this->b = b;
    aPoint = a->WorldSpaceToLocalSpace(anchorA);
    bPoint = b->WorldSpaceToLocalSpace(anchorB);
    m_length = (anchorB - anchorA).Magnitude();
}

void DistanceConstraint::PreSolve(float dt) {
    const Vec2 pa = a->LocalSpaceToWorldSpace(aPoint);
    const Vec2 pb = b->LocalSpaceToWorldSpace(bPoint);
    m_rA = pa - a->position;
    m_rB = pb - b->position;

    Vec2 diff = pb - pa;
    float currentDistance = diff.Magnitude();

    m_n = Vec2(0.0f, 1.0f);
    if (currentDistance > mage::math::EPSILON) {
        m_n = diff / currentDistance;
    }

    float rnA = m_rA.Cross(m_n);
    float rnB = m_rB.Cross(m_n);
    float k = a->invMass + b->invMass + (rnA * rnA) * a->invI + (rnB * rnB) * b->invI;

    m_effectiveMass = k > 0.0f ? 1.0f / k : 0.0f;

    const float beta = 0.02f;
    m_bias = (beta / dt) * (currentDistance - m_length);

    Vec2 impulse = m_n * m_cachedLambda;

    a->ApplyImpulseLinear(impulse * -1.0f);
    a->ApplyImpulseAngular(-rnA * m_cachedLambda);

    b->ApplyImpulseLinear(impulse);
    b->ApplyImpulseAngular(rnB * m_cachedLambda);
}

void DistanceConstraint::Solve() {
    Vec2 vaPoint = a->velocity + Vec2(-a->angularVelocity * m_rA.y, a->angularVelocity * m_rA.x);
    Vec2 vbPoint = b->velocity + Vec2(-b->angularVelocity * m_rB.y, b->angularVelocity * m_rB.x);
    Vec2 rv = vbPoint - vaPoint;

    float jv = rv.Dot(m_n);
    float lambda = m_effectiveMass * (-jv - m_bias);

    m_cachedLambda += lambda;

    Vec2 impulse = m_n * lambda;

    a->ApplyImpulseLinear(impulse * -1.0f);
    a->ApplyImpulseAngular(-m_rA.Cross(m_n) * lambda);

    b->ApplyImpulseLinear(impulse);
    b->ApplyImpulseAngular(m_rB.Cross(m_n) * lambda);
}

void DistanceConstraint::PostSolve() {
    m_cachedLambda = std::clamp(m_cachedLambda, -100000.0f, 100000.0f);
}

// Weld Constraint (Rigid Weld)
WeldConstraint::WeldConstraint()
    : Constraint(), m_cachedImpulse(0.0f, 0.0f, 0.0f), m_bias(0.0f, 0.0f, 0.0f), m_referenceAngle(0.0f) {
}

WeldConstraint::WeldConstraint(Body* a, Body* b, const Vec2& anchorPoint)
    : Constraint(), m_cachedImpulse(0.0f, 0.0f, 0.0f), m_bias(0.0f, 0.0f, 0.0f) {
    this->a = a;
    this->b = b;
    aPoint = a->WorldSpaceToLocalSpace(anchorPoint);
    bPoint = b->WorldSpaceToLocalSpace(anchorPoint);
    m_referenceAngle = b->rotation - a->rotation;
}

void WeldConstraint::PreSolve(float dt) {
    const Vec2 pa = a->LocalSpaceToWorldSpace(aPoint);
    const Vec2 pb = b->LocalSpaceToWorldSpace(bPoint);
    m_rA = pa - a->position;
    m_rB = pb - b->position;

    float mA = a->invMass, mB = b->invMass;
    float iA = a->invI, iB = b->invI;

    Vec3 col1, col2, col3;

    col1.x = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
    col2.x = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
    col3.x = -m_rA.y * iA - m_rB.y * iB;

    col1.y = col2.x;
    col2.y = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
    col3.y = m_rA.x * iA + m_rB.x * iB;

    col1.z = col3.x;
    col2.z = col3.y;
    col3.z = iA + iB;

    m_massMatrix = Mat33(col1, col2, col3);

    const float beta = 0.02f;

    Vec2 C_pos = pb - pa;
    float C_angle = (b->rotation - a->rotation) - m_referenceAngle;

    while (C_angle > mage::math::PI) C_angle -= mage::math::TWO_PI;
    while (C_angle < -mage::math::PI) C_angle += mage::math::TWO_PI;

    m_bias.x = C_pos.x * (beta / dt);
    m_bias.y = C_pos.y * (beta / dt);
    m_bias.z = C_angle * (beta / dt);

    Vec2 P(m_cachedImpulse.x, m_cachedImpulse.y);

    a->ApplyImpulseLinear(P * -1.0f);
    a->ApplyImpulseAngular(-m_rA.Cross(P) - m_cachedImpulse.z);

    b->ApplyImpulseLinear(P);
    b->ApplyImpulseAngular(m_rB.Cross(P) + m_cachedImpulse.z);
}

void WeldConstraint::Solve() {
    Vec2 vaPoint = a->velocity + Vec2(-a->angularVelocity * m_rA.y, a->angularVelocity * m_rA.x);
    Vec2 vbPoint = b->velocity + Vec2(-b->angularVelocity * m_rB.y, b->angularVelocity * m_rB.x);

    Vec2 jvLinear = vbPoint - vaPoint;
    float jvAngular = b->angularVelocity - a->angularVelocity;

    Vec3 Cdot(jvLinear.x, jvLinear.y, jvAngular);

    Vec3 rhs = (Cdot + m_bias) * -1.0f;
    Vec3 impulse = m_massMatrix.Solve33(rhs);

    m_cachedImpulse += impulse;

    Vec2 P(impulse.x, impulse.y);

    a->ApplyImpulseLinear(P * -1.0f);
    a->ApplyImpulseAngular(-m_rA.Cross(P) - impulse.z);

    b->ApplyImpulseLinear(P);
    b->ApplyImpulseAngular(m_rB.Cross(P) + impulse.z);
}

void WeldConstraint::PostSolve() {}

// Prismatic Constraint (Piston / Slider)
PrismaticConstraint::PrismaticConstraint()
    : Constraint(), m_cachedLambda(0.0f, 0.0f), m_bias(0.0f, 0.0f), m_referenceAngle(0.0f) {
}

PrismaticConstraint::PrismaticConstraint(Body* a, Body* b, const Vec2& anchorPoint, const Vec2& axis)
    : Constraint(), m_cachedLambda(0.0f, 0.0f), m_bias(0.0f, 0.0f) {
    this->a = a;
    this->b = b;
    aPoint = a->WorldSpaceToLocalSpace(anchorPoint);
    bPoint = b->WorldSpaceToLocalSpace(anchorPoint);

    m_localAxis = axis.UnitVector().Rotate(-a->rotation);
    m_referenceAngle = b->rotation - a->rotation;
}

void PrismaticConstraint::PreSolve(float dt) {
    const Vec2 pa = a->LocalSpaceToWorldSpace(aPoint);
    const Vec2 pb = b->LocalSpaceToWorldSpace(bPoint);
    m_rA = pa - a->position;
    m_rB = pb - b->position;

    Vec2 worldAxis = m_localAxis.Rotate(a->rotation).UnitVector();
    m_perp = Vec2(-worldAxis.y, worldAxis.x);

    float sA = m_rA.Cross(m_perp);
    float sB = m_rB.Cross(m_perp);

    float mA = a->invMass;
    float mB = b->invMass;
    float iA = a->invI;
    float iB = b->invI;

    float k00 = mA + mB + (sA * sA) * iA + (sB * sB) * iB;
    float k01 = sA * iA + sB * iB;
    float k10 = k01;
    float k11 = iA + iB;

    Mat22 K(k00, k01, k10, k11);
    m_effectiveMass = K.Inverse();

    const float beta = 0.1f;
    const float slop = 0.5f;
    const float angleSlop = 0.01f;

    float C_pos = (pb - pa).Dot(m_perp);
    m_bias.x = (beta / dt) * (std::abs(C_pos) > slop ? C_pos : 0.0f);

    float C_angle = (b->rotation - a->rotation) - m_referenceAngle;
    while (C_angle > mage::math::PI) C_angle -= mage::math::TWO_PI;
    while (C_angle < -mage::math::PI) C_angle += mage::math::TWO_PI;
    m_bias.y = (beta / dt) * (std::abs(C_angle) > angleSlop ? C_angle : 0.0f);

    Vec2 P = m_perp * m_cachedLambda.x;

    a->ApplyImpulseLinear(P * -1.0f);
    a->ApplyImpulseAngular(-sA * m_cachedLambda.x - m_cachedLambda.y);

    b->ApplyImpulseLinear(P);
    b->ApplyImpulseAngular(sB * m_cachedLambda.x + m_cachedLambda.y);
}

void PrismaticConstraint::Solve() {
    Vec2 vaPoint = a->velocity + Vec2(-a->angularVelocity * m_rA.y, a->angularVelocity * m_rA.x);
    Vec2 vbPoint = b->velocity + Vec2(-b->angularVelocity * m_rB.y, b->angularVelocity * m_rB.x);

    float jv_perp = (vbPoint - vaPoint).Dot(m_perp);
    float jv_angle = b->angularVelocity - a->angularVelocity;

    Vec2 jv(jv_perp, jv_angle);

    Vec2 lambda = m_effectiveMass * ((jv + m_bias) * -1.0f);
    m_cachedLambda += lambda;

    Vec2 P = m_perp * lambda.x;
    float sA = m_rA.Cross(m_perp);
    float sB = m_rB.Cross(m_perp);

    a->ApplyImpulseLinear(P * -1.0f);
    a->ApplyImpulseAngular(-sA * lambda.x - lambda.y);

    b->ApplyImpulseLinear(P);
    b->ApplyImpulseAngular(sB * lambda.x + lambda.y);
}

void PrismaticConstraint::PostSolve() {
    m_cachedLambda.x = std::clamp(m_cachedLambda.x, -100000.0f, 100000.0f);
    m_cachedLambda.y = std::clamp(m_cachedLambda.y, -100000.0f, 100000.0f);
}

// Penetration Constraint (Collision Response + Friction)
PenetrationConstraint::PenetrationConstraint()
    : Constraint(), m_normalMass(0.0f), m_tangentMass(0.0f),
    m_normalImpulse(0.0f), m_tangentImpulse(0.0f), m_bias(0.0f), m_friction(0.0f) {
}

PenetrationConstraint::PenetrationConstraint(Body* a, Body* b,
    const Vec2& aCollisionPoint,
    const Vec2& bCollisionPoint,
    const Vec2& normal)
    : Constraint(), m_normalMass(0.0f), m_tangentMass(0.0f),
    m_normalImpulse(0.0f), m_tangentImpulse(0.0f), m_bias(0.0f), m_friction(0.0f) {
    this->a = a;
    this->b = b;
    aPoint = a->WorldSpaceToLocalSpace(aCollisionPoint);
    bPoint = b->WorldSpaceToLocalSpace(bCollisionPoint);
    m_localNormal = a->WorldSpaceToLocalSpace(normal);
}

void PenetrationConstraint::PreSolve(float dt) {
    const Vec2 pa = a->LocalSpaceToWorldSpace(aPoint);
    const Vec2 pb = b->LocalSpaceToWorldSpace(bPoint);

    m_n = a->LocalSpaceToWorldSpace(m_localNormal);
    m_t = m_n.Normal();

    m_rA = pa - a->position;
    m_rB = pb - b->position;

    float rnA = m_rA.Cross(m_n);
    float rnB = m_rB.Cross(m_n);
    float kNormal = a->invMass + b->invMass + (rnA * rnA) * a->invI + (rnB * rnB) * b->invI;
    m_normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

    m_friction = std::max(a->friction, b->friction);
    if (m_friction > 0.0f) {
        float rtA = m_rA.Cross(m_t);
        float rtB = m_rB.Cross(m_t);
        float kTangent = a->invMass + b->invMass + (rtA * rtA) * a->invI + (rtB * rtB) * b->invI;
        m_tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;
    }

    float penetration = (pb - pa).Dot(-m_n);
    float C = std::min(0.0f, penetration + mage::physics::PENETRATION_SLOP);
    m_bias = (mage::physics::BAUMGARTE_BETA / dt) * C;

    Vec2 impulse = m_n * m_normalImpulse + m_t * m_tangentImpulse;

    a->ApplyImpulseLinear(-impulse);
    a->ApplyImpulseAngular(-m_rA.Cross(impulse));

    b->ApplyImpulseLinear(impulse);
    b->ApplyImpulseAngular(m_rB.Cross(impulse));
}

void PenetrationConstraint::Solve() {
    Vec2 vaPoint = a->velocity + Vec2(-a->angularVelocity * m_rA.y, a->angularVelocity * m_rA.x);
    Vec2 vbPoint = b->velocity + Vec2(-b->angularVelocity * m_rB.y, b->angularVelocity * m_rB.x);
    Vec2 rv = vbPoint - vaPoint;

    float vn = rv.Dot(m_n);

    float dPn = m_normalMass * (-vn - m_bias);

    float oldNormalImpulse = m_normalImpulse;
    m_normalImpulse = std::max(oldNormalImpulse + dPn, 0.0f);
    dPn = m_normalImpulse - oldNormalImpulse;

    Vec2 Pn = m_n * dPn;
    a->ApplyImpulseLinear(-Pn);
    a->ApplyImpulseAngular(-m_rA.Cross(Pn));
    b->ApplyImpulseLinear(Pn);
    b->ApplyImpulseAngular(m_rB.Cross(Pn));

    if (m_friction > 0.0f) {
        vaPoint = a->velocity + Vec2(-a->angularVelocity * m_rA.y, a->angularVelocity * m_rA.x);
        vbPoint = b->velocity + Vec2(-b->angularVelocity * m_rB.y, b->angularVelocity * m_rB.x);
        rv = vbPoint - vaPoint;

        float vt = rv.Dot(m_t);
        float dPt = m_tangentMass * (-vt);

        float maxFriction = m_friction * m_normalImpulse;

        float oldTangentImpulse = m_tangentImpulse;
        m_tangentImpulse = std::clamp(oldTangentImpulse + dPt, -maxFriction, maxFriction);
        dPt = m_tangentImpulse - oldTangentImpulse;

        Vec2 Pt = m_t * dPt;
        a->ApplyImpulseLinear(-Pt);
        a->ApplyImpulseAngular(-m_rA.Cross(Pt));
        b->ApplyImpulseLinear(Pt);
        b->ApplyImpulseAngular(m_rB.Cross(Pt));
    }
}

void PenetrationConstraint::PostSolve() {}