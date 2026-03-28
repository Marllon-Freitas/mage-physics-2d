/**
 * @file World.h
 * @brief The core physics simulation environment and orchestrator.
 *
 * The World class is the heart of the M.A.G.E. physics engine. It owns and manages all
 * active rigid bodies and constraints, drives the main physics step,
 * handles the collision detection pipeline (broad and narrow phases), and
 * commands the solver to resolve physical interactions and resting states.
 */

#ifndef WORLD_H
#define WORLD_H

#include "../Solver/Manifold.h"
#include "../Solver/Constraint.h"
#include "../Collision/CollisionDetection.h"
#include "../Collision/SpatialHashGrid.h"
#include "Body.h"

#include <iostream>
#include <memory>
#include <vector>
#include <map>

 /**
  * @struct WorldSettings
  * @brief Configuration parameters for initializing the physics world.
  * Allows the external engine (or ECS) to define the scale and quality of the simulation.
  */
struct WorldSettings {
    float gravity = -9.8f;          // Global gravity applied to dynamic bodies.
    float cellSize = 100.0f;        // Spatial Hash Grid cell size. (Should be ~2x the average object size).
    size_t poolCapacity = 10000;    // Pre-allocated capacity for the DOD body pool to avoid heap allocations.
    int solverIterations = 20;      // Number of Gauss-Seidel iterations. Higher = more stable joints/stacks.

    // Sleep thresholds
    float sleepLinearThresholdSq = 100.0f; // Squared linear velocity threshold to consider sleeping.
    float sleepAngularThresholdSq = 0.5f;  // Squared angular velocity threshold to consider sleeping.
    float sleepTimeRequired = 0.5f;        // Time (in seconds) bodies must remain below thresholds to fall asleep.
};

/**
 * @struct RaycastOptions
 * @brief Options to filter and customize raycast queries.
 */
struct RaycastOptions {
    bool  ignoreSensors = true;     // If true, the ray will pass through sensor bodies.
    bool  ignoreSleeping = false;   // If true, the ray ignores bodies that are currently sleeping.
    Body* ignoreBody = nullptr;     // Specific body to ignore (usually the entity casting the ray).
};

/**
 * @struct CollisionEvent
 * @brief Represents a new contact point between two bodies.
 */
struct CollisionEvent {
    Body* a;
    Body* b;
    Vec2  normal;
    float depth;
    float impactSpeed;
};

/**
 * @struct SeparationEvent
 * @brief Emitted when two bodies stop touching.
 */
struct SeparationEvent {
    Body* a;
    Body* b;
};

/**
 * @class World
 * @brief The main physics simulation environment.
 */
class World {
public:
    /**
     * @brief Constructs a new physics world with the given settings.
     * @param settings The configuration object defining gravity, memory limits, and solver quality.
     */
    explicit World(const WorldSettings& settings);
    ~World() = default;

    /**
     * @brief Creates a body directly in the contiguous DOD memory pool.
     * @return Pointer to the newly created Body, or nullptr if pool capacity is reached.
     */
    template<typename... Args>
    Body* CreateBody(Args&&... args) {
        if (m_bodyPool.size() >= m_bodyPool.capacity()) {
            std::cerr << "[PHYSICS FATAL] Memory pool limit reached! Increase poolCapacity." << std::endl;
            return nullptr;
        }

        m_bodyPool.emplace_back(std::forward<Args>(args)...);

        Body* newBody = &m_bodyPool.back();
        m_bodies.push_back(newBody);

        return newBody;
    }

    std::vector<Body*>& GetBodies();

    // Constraint management
    void AddConstraint(std::unique_ptr<Constraint> constraint);
    void RemoveConstraint(Constraint* constraint);
    const std::vector<Constraint*>& GetConstraints() const;

    // Manifold access
    std::map<std::pair<Body*, Body*>, Manifold>& GetManifolds();

    // Event bus
    const std::vector<CollisionEvent>& GetCollisionEvents() const { return m_collisionEvents; }
    const std::vector<SeparationEvent>& GetSeparationEvents() const { return m_separationEvents; }
    void ClearEvents() {
        m_collisionEvents.clear();
        m_separationEvents.clear();
    }

    // Global forces
    void AddForce(const Vec2& force);
    void AddTorque(float torque);

    /**
     * @brief Steps the physics simulation forward by deltaTime.
     * @param dt The time step.
     */
    void Update(float dt);

    /**
     * @brief Casts a ray through the world and returns the closest intersection.
     */
    RaycastResult Raycast(const Vec2& start, const Vec2& end, const RaycastOptions& options = RaycastOptions{});

private:
    WorldSettings m_settings; // Active settings for the simulation.

    // Contiguous DOD memory pool — no heap fragmentation
    std::vector<Body> m_bodyPool;
    std::vector<Body*> m_bodies;

    std::vector<std::unique_ptr<Constraint>> m_constraints; // Owners
    std::vector<Constraint*> m_constraintObservers;         // Cache for rendering/external reading

    // Persistent collision manifolds across frames
    std::map<std::pair<Body*, Body*>, Manifold> m_manifolds;

    // The Event Bus: Queues for events processed this frame
    std::vector<CollisionEvent> m_collisionEvents;
    std::vector<SeparationEvent> m_separationEvents;

    // Global forces and torques applied this frame
    std::vector<Vec2>  m_forces;
    std::vector<float> m_torques;

    SpatialHashGrid m_spatialGrid;

    // Internal pipeline stages
    void ApplyForces();
    std::vector<std::pair<Body*, Body*>> BroadPhase();
    void NarrowPhase(const std::vector<std::pair<Body*, Body*>>& candidatePairs);
    void DetectCollisions();
    void SolveConstraints(float dt);
    void UpdateIslandSleeps(float dt);
};

#endif