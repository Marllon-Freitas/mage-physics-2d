#include "World.h"
#include "PhysicsConstants.h"
#include "../Solver/Constraint.h"

#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <utility>
#include <cassert>

World::World(const WorldSettings& settings)
    : m_settings(settings),
    m_spatialGrid(settings.cellSize)
{
    // We invert the gravity sign internally so -9.8 pulls down on the Y axis
    m_settings.gravity = -settings.gravity;
    m_bodyPool.reserve(m_settings.poolCapacity);

    m_collisionEvents.reserve(64);
    m_separationEvents.reserve(64);
}

// Body & constraint access
std::vector<Body*>& World::GetBodies() {
    return m_bodies;
}

void World::AddConstraint(std::unique_ptr<Constraint> constraint) {
    if (constraint) {
        m_constraintObservers.push_back(constraint.get());
        m_constraints.push_back(std::move(constraint));

        assert(m_constraints.size() == m_constraintObservers.size() && "FATAL: Constraint cache desync!");
    }
}

void World::RemoveConstraint(Constraint* constraint) {
    // Remove from observers cache first to prevent dangling pointer
    auto obsIt = std::find(m_constraintObservers.begin(), m_constraintObservers.end(), constraint);
    if (obsIt != m_constraintObservers.end()) {
        m_constraintObservers.erase(obsIt);
    }

    // Remove from unique_ptr owner vector (automatically deletes memory)
    auto it = std::remove_if(m_constraints.begin(), m_constraints.end(),
        [constraint](const std::unique_ptr<Constraint>& ptr) {
            return ptr.get() == constraint;
        });

    if (it != m_constraints.end()) {
        m_constraints.erase(it, m_constraints.end());
    }

    assert(m_constraints.size() == m_constraintObservers.size() && "FATAL: Constraint cache desync!");
}

const std::vector<Constraint*>& World::GetConstraints() const {
    return m_constraintObservers;
}

std::map<std::pair<Body*, Body*>, Manifold>& World::GetManifolds() {
    return m_manifolds;
}

// Global forces
void World::AddForce(const Vec2& force) {
    m_forces.push_back(force);
}

void World::AddTorque(float torque) {
    m_torques.push_back(torque);
}

void World::ApplyForces() {
    for (auto* body : m_bodies) {
        if (body->IsStatic() || body->isSleeping || body->IsKinematic()) continue;

        // Apply gravity scaled to simulation units (P = m * g)
        Vec2 weight = Vec2(0.0f, body->mass * m_settings.gravity * mage::physics::PIXELS_PER_METER);
        body->AddForce(weight);

        // Apply external global forces
        for (const auto& force : m_forces)
            body->AddForce(force);

        for (float torque : m_torques)
            body->AddTorque(torque);
    }
}

// Collision pipeline
std::vector<std::pair<Body*, Body*>> World::BroadPhase() {
    m_spatialGrid.Clear();

    for (auto* body : m_bodies)
        m_spatialGrid.Insert(body);

    return m_spatialGrid.GetCandidatePairs();
}

void World::NarrowPhase(const std::vector<std::pair<Body*, Body*>>& candidatePairs) {
    std::map<std::pair<Body*, Body*>, Manifold> newManifolds;

    for (const auto& pair : candidatePairs) {
        Body* a = pair.first;
        Body* b = pair.second;

        std::vector<Contact> contacts;
        if (!CollisionDetection::IsColliding(a, b, contacts)) continue;

        // Canonicalize pair ordering for stable manifold keys
        bool  swapped = (a > b);
        Body* body1 = swapped ? b : a;
        Body* body2 = swapped ? a : b;
        auto  key = std::make_pair(body1, body2);

        if (swapped) {
            for (auto& c : contacts) {
                std::swap(c.a, c.b);
                std::swap(c.start, c.end);
                c.normal = -c.normal;
            }
        }

        // One-way platform logic
        if (body1->isOneWay || body2->isOneWay) {
            Body* platform = body1->isOneWay ? body1 : body2;
            Body* other = body1->isOneWay ? body2 : body1;

            Vec2 n = contacts[0].normal;
            if (body2->isOneWay) n = -n;

            // Reject if normal doesn't point up, or if other body is moving upward
            if (n.y > -0.1f || other->velocity.y < 0.0f) continue;
        }

        bool isNewCollision = (m_manifolds.find(key) == m_manifolds.end());
        bool body1AwakeAndActive = !body1->isSleeping && !body1->IsStatic();
        bool body2AwakeAndActive = !body2->isSleeping && !body2->IsStatic();

        // Wake up bodies on new collision or if either is already active
        if (isNewCollision || body1AwakeAndActive || body2AwakeAndActive) {
            body1->WakeUp();
            body2->WakeUp();
        }

        // Fire enter callbacks and Event Bus on the first frame of contact
        if (isNewCollision) {
            // Calculate relative speed along the collision normal
            Vec2 relativeVel = body1->velocity - body2->velocity;
            float impactSpeed = relativeVel.Dot(contacts[0].normal);

            // Only consider positive approaching speeds
            impactSpeed = std::max(0.0f, impactSpeed);

            // Feed the Event Bus
            m_collisionEvents.push_back({ body1, body2, contacts[0].normal, contacts[0].depth, impactSpeed });
        }

        // Update persistent manifold or create a new one
        if (!isNewCollision) {
            Manifold old = m_manifolds[key];
            old.Update(contacts);
            newManifolds[key] = old;
        }
        else {
            Manifold fresh(body1, body2);
            fresh.Update(contacts);
            newManifolds[key] = fresh;
        }
    }

    // Fire exit callbacks for manifolds that no longer exist this frame
    for (auto& [key, oldManifold] : m_manifolds) {
        if (newManifolds.find(key) == newManifolds.end()) {
            Body* b1 = key.first;
            Body* b2 = key.second;

            // Feed the Event Bus
            m_separationEvents.push_back({ b1, b2 });
        }
    }

    m_manifolds = newManifolds;
}

void World::DetectCollisions() {
    auto candidatePairs = BroadPhase();
    NarrowPhase(candidatePairs);
}

// Island sleeping
void World::UpdateIslandSleeps(float dt) {
    std::unordered_map<Body*, std::vector<Body*>> adjList;

    // Connect dynamically active bodies via manifolds
    for (auto& [key, manifold] : m_manifolds) {
        Body* a = manifold.a;
        Body* b = manifold.b;

        if (!a->IsStatic() && !b->IsStatic()) {
            adjList[a].push_back(b);
            adjList[b].push_back(a);
        }
    }

    // Connect dynamically active bodies via constraints
    for (auto& constraint : m_constraints) {
        Body* a = constraint->a;
        Body* b = constraint->b;

        if (!a->IsStatic() && !b->IsStatic()) {
            adjList[a].push_back(b);
            adjList[b].push_back(a);
        }
    }

    std::unordered_set<Body*> visited;

    for (auto* body : m_bodies) {
        if (body->IsStatic() || visited.count(body)) continue;

        std::vector<Body*> island;
        std::vector<Body*> stack;

        stack.push_back(body);
        visited.insert(body);

        while (!stack.empty()) {
            Body* curr = stack.back();
            stack.pop_back();
            island.push_back(curr);

            for (Body* neighbor : adjList[curr]) {
                if (!visited.count(neighbor)) {
                    visited.insert(neighbor);
                    stack.push_back(neighbor);
                }
            }
        }

        bool islandCanSleep = true;
        for (Body* b : island) {
            bool tooFast = b->velocity.MagnitudeSquared() > m_settings.sleepLinearThresholdSq
                || (b->angularVelocity * b->angularVelocity) > m_settings.sleepAngularThresholdSq;

            if (tooFast) {
                islandCanSleep = false;
                break;
            }
        }

        if (islandCanSleep) {
            for (Body* b : island) {
                b->sleepTimer += dt;
                if (b->sleepTimer >= m_settings.sleepTimeRequired) {
                    b->isSleeping = true;
                    b->velocity = Vec2(0.0f, 0.0f);
                    b->angularVelocity = 0.0f;
                }
            }
        }
        else {
            for (Body* b : island) {
                b->sleepTimer = 0.0f;
                b->isSleeping = false;
            }
        }
    }
}

// Constraint solver (PreSolve → Solve × N → PostSolve)
void World::SolveConstraints(float dt) {
    auto ShouldSkipManifold = [&](const Manifold& manifold) -> bool {
        if (manifold.a->isSensor || manifold.b->isSensor) return true;
        if (manifold.a->mass == 0.0f && manifold.b->mass == 0.0f) return true;
        if ((manifold.a->isSleeping || manifold.a->IsStatic()) &&
            (manifold.b->isSleeping || manifold.b->IsStatic())) return true;

        for (auto& c : m_constraints) {
            if ((c->a == manifold.a && c->b == manifold.b) ||
                (c->a == manifold.b && c->b == manifold.a)) return true;
        }

        return false;
        };

    auto BothConstraintBodiesInactive = [](const Constraint* c) -> bool {
        return (c->a->isSleeping || c->a->IsStatic()) &&
            (c->b->isSleeping || c->b->IsStatic());
        };

    // --- PreSolve ---
    for (auto& constraint : m_constraints) {
        if (BothConstraintBodiesInactive(constraint.get())) continue;
        constraint->PreSolve(dt);
    }

    for (auto& [key, manifold] : m_manifolds) {
        if (ShouldSkipManifold(manifold)) continue;
        manifold.PreSolve(dt);
    }

    // --- Solve ---
    for (int i = 0; i < m_settings.solverIterations; i++) {
        for (auto& constraint : m_constraints) {
            if (BothConstraintBodiesInactive(constraint.get())) continue;
            constraint->Solve();
        }

        for (auto& [key, manifold] : m_manifolds) {
            if (ShouldSkipManifold(manifold)) continue;
            manifold.Solve();
        }
    }

    // --- PostSolve ---
    for (auto& constraint : m_constraints) {
        if (BothConstraintBodiesInactive(constraint.get())) continue;
        constraint->PostSolve();
    }

    for (auto& [key, manifold] : m_manifolds) {
        if (ShouldSkipManifold(manifold)) continue;
        manifold.PostSolve();
    }
}

// Spatial queries
RaycastResult World::Raycast(const Vec2& start, const Vec2& end, const RaycastOptions& options) {
    RaycastResult closest;
    closest.t = 1.0f;

    Vec2  rayDir = end - start;
    float rayLenSq = rayDir.MagnitudeSquared();

    if (rayLenSq < mage::math::EPSILON_SQ) return closest;

    for (auto* body : m_bodies) {
        if (body == options.ignoreBody) continue;
        if (options.ignoreSensors && body->isSensor) continue;
        if (options.ignoreSleeping && body->isSleeping) continue;

        // Broad phase: reject bodies whose center is far from the ray
        Vec2  toBody = body->position - start;
        float dot = toBody.Dot(rayDir);
        float clampedT = std::clamp(dot / rayLenSq, 0.0f, 1.0f);
        Vec2  closestPoint = start + rayDir * clampedT;
        Vec2  diff = body->position - closestPoint;

        if (diff.MagnitudeSquared() > body->boundingRadius * body->boundingRadius) continue;

        // Narrow phase: exact shape intersection
        RaycastResult temp;
        bool hit = false;

        ShapeType type = body->shape->GetType();
        if (type == CIRCLE) hit = CollisionDetection::RaycastCircle(body, start, end, temp);
        else if (type == BOX || type == POLYGON) hit = CollisionDetection::RaycastPolygon(body, start, end, temp);
        else if (type == CAPSULE) hit = CollisionDetection::RaycastCapsule(body, start, end, temp);

        if (hit && temp.t < closest.t)
            closest = temp;
    }

    return closest;
}

// Main update loop
void World::Update(float dt) {
    // Clear events from the previous frame to prepare for the new simulation step
    ClearEvents();

    // Apply Gravity & Global Forces
    ApplyForces();

    // Integrate Forces (Velocity update)
    for (auto* body : m_bodies) {
        if (!body->IsStatic() && !body->isSleeping)
            body->IntegrateForces(dt);
    }

    // Generate Manifolds & Feed Event Bus
    DetectCollisions();

    // Iterative Solver
    SolveConstraints(dt);

    // Integrate Velocities (Position update & Damping)
    for (auto* body : m_bodies) {
        if (!body->IsStatic() && !body->isSleeping)
            body->IntegrateVelocities(dt);
    }

    // Graph-theory Island Sleep Optimization
    UpdateIslandSleeps(dt);
}