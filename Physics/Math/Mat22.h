/**
 * @file Mat22.h
 * @brief A fast, stack-allocated 2x2 matrix for physics solver calculations.
 */

#ifndef MAT22_H
#define MAT22_H

#include "Vec2.h"

struct Mat22 {
    float m00, m01;
    float m10, m11;

    constexpr Mat22() : m00(0.0f), m01(0.0f), m10(0.0f), m11(0.0f) {}
    constexpr Mat22(float m00, float m01, float m10, float m11)
        : m00(m00), m01(m01), m10(m10), m11(m11) {
    }

    /**
     * @brief Matrix-vector multiplication
     */
    constexpr Vec2 operator*(const Vec2& v) const {
        return Vec2(m00 * v.x + m01 * v.y, m10 * v.x + m11 * v.y);
    }

    /**
     * @brief Calculates the inverse of the matrix (used in PreSolve phase)
     */
    Mat22 Inverse() const {
        float det = m00 * m11 - m01 * m10;

        if (det != 0.0f) {
            float invDet = 1.0f / det;
            return Mat22(m11 * invDet, -m01 * invDet,
                -m10 * invDet, m00 * invDet);
        }

        // Return zero matrix in case of singularity
        return Mat22();
    }
};

#endif