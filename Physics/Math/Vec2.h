/**
 * @file Vec2.h
 * @brief 2D vector for positions, forces, and velocities.
 */

#ifndef VEC2_H
#define VEC2_H

#include "../Core/PhysicsConstants.h"

#include <cmath>
#include <cassert>

struct Vec2 {
    float x;
    float y;

    constexpr Vec2() : x(0.0f), y(0.0f) {}
    constexpr Vec2(float x, float y) : x(x), y(y) {}

    Vec2 Rotate(float angle) const;
    float Magnitude() const;

    /** @brief Squared magnitude avoids the costly square root operation. */
    constexpr float MagnitudeSquared() const {
        return (x * x) + (y * y);
    }

    Vec2& Normalize();
    Vec2 UnitVector() const;
    Vec2 Normal() const;

    constexpr float Dot(const Vec2 v) const {
        return (x * v.x) + (y * v.y);
    }

    constexpr float Cross(const Vec2 v) const {
        return (x * v.y) - (y * v.x);
    }

    bool operator==(const Vec2 v) const {
        return std::fabs(x - v.x) < mage::math::EPSILON && std::fabs(y - v.y) < mage::math::EPSILON;
    }

    bool operator!=(const Vec2 v) const {
        return !(*this == v);
    }

    constexpr Vec2 operator+(const Vec2 v) const { return Vec2(x + v.x, y + v.y); }
    constexpr Vec2 operator-(const Vec2 v) const { return Vec2(x - v.x, y - v.y); }
    constexpr Vec2 operator*(float n)       const { return Vec2(x * n, y * n); }

    Vec2 operator/(float n) const {
        assert(n != 0.0f && "Vec2 division by zero");
        float inv = 1.0f / n;
        return Vec2(x * inv, y * inv);
    }

    constexpr Vec2 operator-() const { return Vec2(-x, -y); }

    Vec2& operator+=(const Vec2 v) { x += v.x; y += v.y; return *this; }
    Vec2& operator-=(const Vec2 v) { x -= v.x; y -= v.y; return *this; }
    Vec2& operator*=(float n) { x *= n; y *= n; return *this; }

    Vec2& operator/=(float n) {
        assert(n != 0.0f && "Vec2 division by zero");
        float inv = 1.0f / n;
        x *= inv; y *= inv;
        return *this;
    }
};

/** @brief Scalar multiplication: n * v */
inline constexpr Vec2 operator*(float n, const Vec2 v) {
    return Vec2(v.x * n, v.y * n);
}

#endif