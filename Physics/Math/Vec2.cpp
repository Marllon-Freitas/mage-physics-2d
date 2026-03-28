#include "Vec2.h"

Vec2 Vec2::Rotate(float angle) const {
    float c = std::cos(angle);
    float s = std::sin(angle);
    return Vec2(x * c - y * s, x * s + y * c);
}

float Vec2::Magnitude() const {
    return std::sqrt(MagnitudeSquared());
}

Vec2& Vec2::Normalize() {
    float lengthSq = MagnitudeSquared();
    if (lengthSq > mage::math::EPSILON_SQ) {
        float invLength = 1.0f / std::sqrt(lengthSq);
        x *= invLength;
        y *= invLength;
    }
    return *this;
}

Vec2 Vec2::UnitVector() const {
    float lengthSq = MagnitudeSquared();
    if (lengthSq > mage::math::EPSILON_SQ) {
        float invLength = 1.0f / std::sqrt(lengthSq);
        return Vec2(x * invLength, y * invLength);
    }
    return Vec2(0.0f, 0.0f);
}

Vec2 Vec2::Normal() const {
    // Perpendicular left-hand normal
    return Vec2(y, -x).UnitVector();
}