/**
 * @file Force.h
 * @brief Generators for various physical forces (Drag, Friction, Springs, Gravity).
 * * Provides static utility functions to calculate and return specific force vectors
 * to be applied to rigid bodies during the physics simulation step.
 */

#ifndef FORCE_H
#define FORCE_H

#include "../Math/Vec2.h"
#include "../Core/Body.h"

struct Force {
	static Vec2 GenerateDragForce(const Body& body, float k);
	static Vec2 GenerateFrictionForce(const Body& body, float k);
	static Vec2 GenerateSpringForce(const Body& body, Vec2 anchor, float restLength, float k);
	static Vec2 GenerateSpringForce(const Body& a, const Body& b, float restLength, float k);
	static Vec2 GenerateGravitationalForce(const Body& a, const Body& b, float G, float minDistance, float maxDistance);
};

#endif