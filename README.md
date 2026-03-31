# Game AI Programming Project

This repository contains a collection of AI implementations developed as part of the **Game AI Programming** course in the fourth semester at Digital Arts & Entertainment (DAE) of the major Game Development (GD). The project explores the fundamental algorithms used to create the "illusion of intelligence" in games, focusing on movement, pathfinding, and decision-making within a high-performance C++ and Unreal Engine 5 environment.

## Core Features

### 1. Steering Behaviors & Movement
Implementation of classic steering behaviors for autonomous agents, focusing on smooth and realistic movement.
* **Single Steering Behaviors:** Individual behaviors such as Seek, Flee, Arrive, Evade, Persuit, and Wander.
* **Combined Steering (Flocking):** Complex group dynamics achieved through two main blending methods:
    * **Weighted Combined:** Balancing multiple behaviors (Separation, Cohesion, Alignment) through adjustable weights to create a "Flocking" algorithm.
    * **Priority Combined:** A hierarchical approach where critical behaviors (like collision avoidance) take precedence over lower-priority ones.

### 2. Pathfinding & Graph Theory
Efficient navigation through complex environments using various search algorithms.
* **Breadth-First Search (BFS):** Unweighted graph traversal for finding the shortest path in terms of steps.
* **A* (A-Star):** A heuristic-based approach for optimized, weighted pathfinding.

### 3. Navigation Meshes (NavMesh)
Advanced navigation using mesh-based representations of the environment.
* **NavMesh Integration:** Utilizing the A* algorithm over a navigation mesh for more natural movement across polygons.
* **Simple Stupid Funnel Algorithm (SSFA):** Implemented as a post-processing step for the NavMesh to optimize paths, "tightening" the route around corners to produce realistic, straight-line movement.

## Technical Stack
* **Engine:** Unreal Engine 5 (Top-down framework).
* **Language:** C++ (All core AI logic is implemented in C++ for performance).
* **UI/Debugging:** ImGui integration for real-time parameter tweaking and visual debugging of paths and steering forces.
* **Framework Components:**
    * `BaseAgent`: The core API for all autonomous actors.
    * `WorldTrimVolume`: Boundary logic to keep agents within the simulation bounds.
    * `GameAISpectatorController`: Custom input handling for world interaction.

## Project Structure
The project is organized to maintain a clean separation between engine content and C++ logic:
* **C++ Source:** Contains the mathematical implementation of steering, graph searching, and the Funnel algorithm.
* **Level Blueprints:** Minimal wrappers used to coordinate actions and modify C++ properties for specific scenarios.

## About the Course
This project is part of the **Game AI Programming** curriculum in semester 4 at Howest DAE in the major Game Development (GD). 

---
*Note: This repository is currently in development. The final Exam Project implementation is TBD.*
