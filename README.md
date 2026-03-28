# M.A.G.E. Physics 2D
A simple, impulse-based 2D physics engine written in C++. Designed as a high-performance educational core, modular and agnostic to rendering engines.

---

## Technical Highlights

*   **Memory Pooling:** Pre-allocated body storage in contiguous memory to prevent heap fragmentation and ensure pointer stability during the simulation step.
*   **Persistent Manifolds and Warm Starting:** Stores impulse history between frames, significantly increasing stability for object stacks and complex constraints.
*   **Broad-Phase Optimization:** Spatial Hash Grid implementation to filter collision pairs, providing sub-linear performance in sparse scenes compared to $O(n^2)$ brute-force checks.
*   **Narrow-Phase Algorithms:** Robust implementation of the Separating Axis Theorem (SAT) for convex polygons, with specialized analytical tests for Circles and Capsules.

---

## Core Features

### Geometry and Shapes
*   **Circles:** Fast analytical collisions for projectiles and spherical objects.
*   **Boxes (OBB):** Oriented Bounding Boxes with full rotation support.
*   **Convex Polygons:** Custom shapes with automatic calculation of area, centroid, and moment of inertia.
*   **Capsules:** Optimized for character controllers to prevent snagging on environment geometry.

### Constraint System
Impulse-based sequential solver (Gauss-Seidel) for mechanical joints:
*   **Joint (Pivot):** Connects two bodies at a shared anchor point.
*   **Distance:** Maintains a fixed distance between bodies (rigid rope/rod behavior).
*   **Weld:** Locks two bodies together, maintaining relative position and orientation.

### Stability and Performance
*   **Island Sleep:** Graph-based optimization (DFS) that identifies resting object groups and deactivates them to save CPU cycles.
*   **One-Way Platforms:** Native support for ghost-collision logic from specific directions.
*   **Sensors (Triggers):** Collision detection without physical response for gameplay triggers.
*   **Raycasting:** Precision spatial queries for line-of-sight or projectile pathing.

---

## Project Structure

*   `/Core`: World orchestration, Body definition, and global constants.
*   `/Collision`: Geometric algorithms, SpatialHashGrid, and Shape definitions.
*   `/Math`: Custom linear algebra library (Vec2, Mat22, Mat33).
*   `/Solver`: Impulse resolution, Manifolds, and Constraints.

## License
This project is licensed under the MIT License.

---
