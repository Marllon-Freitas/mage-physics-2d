/**
 * @file Manifold.h
 * @brief Manages persistent contact information between two colliding bodies.
 * * The Manifold acts as the "memory" of the physics engine. It tracks contact
 * points across multiple frames, allowing the constraint solver to cache impulses
 * (Warm Starting). This is critical for stable stacking and preventing jitter.
 */

#ifndef MANIFOLD_H
#define MANIFOLD_H

#include "../Solver/Constraint.h"
#include "../Collision/Contact.h"
#include "../Core/PhysicsConstants.h"

#include <vector>

class Manifold {
public:
    Body* a;
    Body* b;
    int numContacts;

    Contact contacts[mage::physics::MAX_CONTACTS];
    PenetrationConstraint constraints[mage::physics::MAX_CONTACTS];

    Manifold() : a(nullptr), b(nullptr), numContacts(0) {}
    Manifold(Body* a, Body* b) : a(a), b(b), numContacts(0) {}

    /**
    * @brief Refreshes the active contacts, preserving cached impulses for recurring points.
    * @param newContacts The list of contact points detected in the current frame.
    */
    void Update(const std::vector<Contact>& newContacts);

    void PreSolve(float dt);
    void Solve();
    void PostSolve();
};

#endif