/**
 * @file Mat33.h
 * @brief A fast, stack-allocated 3x3 matrix and 3D vector for solver calculations.
 */

#ifndef MAT33_H
#define MAT33_H

#include "Vec2.h"

 /**
  * @brief A lightweight 3D vector, primarily used alongside Mat33 for Weld constraints.
  */
struct Vec3 {
    float x, y, z;

    constexpr Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
    constexpr Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3  operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3  operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3  operator*(float s)       const { return Vec3(x * s, y * s, z * s); }
    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
};

/**
 * @brief A static, high-performance 3x3 matrix.
 */
struct Mat33 {
    Vec3 ex, ey, ez; // Matrix columns

    constexpr Mat33() {}
    constexpr Mat33(const Vec3& col1, const Vec3& col2, const Vec3& col3)
        : ex(col1), ey(col2), ez(col3) {
    }

    /**
     * @brief Solves the equation A * x = b, returning 'x' without inverting the entire matrix.
     * Uses Cramer's Rule for fast and stable resolution in physics constraints.
     */
    Vec3 Solve33(const Vec3& b) const {
        float det = ex.x * (ey.y * ez.z - ez.y * ey.z)
            + ex.y * (ez.x * ey.z - ey.x * ez.z)
            + ex.z * (ey.x * ez.y - ez.x * ey.y);

        if (det != 0.0f) {
            float invDet = 1.0f / det;
            Vec3 x;
            x.x = invDet * (b.x * (ey.y * ez.z - ez.y * ey.z)
                + b.y * (ez.x * ey.z - ey.x * ez.z)
                + b.z * (ey.x * ez.y - ez.x * ey.y));

            x.y = invDet * (ex.x * (b.y * ez.z - ez.y * b.z)
                + ex.y * (ez.x * b.z - b.x * ez.z)
                + ex.z * (b.x * ez.y - ez.x * b.y));

            x.z = invDet * (ex.x * (ey.y * b.z - b.y * ey.z)
                + ex.y * (b.x * ey.z - ey.x * b.z)
                + ex.z * (ey.x * b.y - b.x * ey.y));
            return x;
        }

        // Return zero vector in case of singularity
        return Vec3(0.0f, 0.0f, 0.0f);
    }
};

#endif