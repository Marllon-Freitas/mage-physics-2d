/**
 * @file Contact.h
 * @brief Data structure representing a contact point between two colliding bodies, including penetration depth and collision normal.
 */

#ifndef CONTACT_H
#define CONTACT_H

#include "../Math/Vec2.h"
#include "../Core/Body.h"

struct Contact {
    Body* a;
    Body* b;

    Vec2 start;
    Vec2 end;

    Vec2 normal;
    float depth;

    Contact()
        : a(nullptr), b(nullptr), start(), end(), normal(), depth(0.0f) {
    }
};

#endif