# Multi-Algorithm Path Planning and Trajectory Optimization for Autonomous Marine Vehicles

A research-oriented simulation framework for **path planning, trajectory smoothing, feasibility analysis, and path tracking for autonomous marine vehicles in cluttered 2D environments**.

The project implements and compares multiple classical and sampling-based path-planning algorithms within a common simulation environment.

The current framework includes:

- Dijkstra
- A*
- RRT*
- Heading-aware Hybrid A*-style lattice planning
- Spline-based path smoothing
- Collision checking
- Curvature analysis
- Obstacle-clearance analysis
- Pure Pursuit-style path tracking
- Quantitative planner benchmarking

The main objective is not only to generate collision-free paths, but also to evaluate whether the resulting paths are **short, smooth, computationally efficient, obstacle-aware, and practically trackable**.

---

# 1. Motivation

Path planning is a fundamental component of autonomous navigation systems.

For an autonomous marine vehicle, finding a collision-free path is only the first part of the problem.

A useful trajectory should also provide:

```text
Collision Avoidance
        +
Short Path Length
        +
Obstacle Clearance
        +
Low Curvature
        +
Smooth Geometry
        +
Computational Efficiency
        +
Trackability
```

Different planning algorithms provide different trade-offs between these objectives.

For example:

```text
Grid-Based Search
      ↓
Deterministic and Structured

Sampling-Based Planning
      ↓
Flexible Search in Continuous Space

Heading-Aware Planning
      ↓
Improved Kinematic Feasibility
```

This project provides a common framework for comparing these approaches.

---

# 2. System Architecture

The overall architecture is:

```text
Environment / Occupancy Map
            ↓
      Start and Goal
            ↓
     Path Planning
  ┌─────────┼─────────┬─────────────┐
  ↓         ↓         ↓             ↓
Dijkstra    A*       RRT*       Hybrid A*
  └─────────┴─────────┴─────────────┘
            ↓
      Raw Path
            ↓
     Path Smoothing
            ↓
     Collision Check
            ↓
    Curvature Analysis
            ↓
     Path Tracking
            ↓
 Performance Evaluation
```

This structure separates the problem into:

```text
Planning
   ↓
Post-Processing
   ↓
Feasibility Analysis
   ↓
Tracking
   ↓
Benchmarking
```

---

# 3. Environment Representation

The current implementation uses a two-dimensional occupancy-grid environment.

Each grid cell is classified as:

```text
0 → Free Space
1 → Obstacle
```

The planner searches for a valid path between:

```text
Start Position
      ↓
Free Navigable Space
      ↓
Goal Position
```

while avoiding occupied cells.

The environment is generic and designed for algorithmic evaluation rather than representing a specific geographic location.

---

# 4. Dijkstra Algorithm

Dijkstra's algorithm is implemented as the first deterministic baseline.

For every node:

```text
g(n)
```

represents the accumulated path cost from the starting position.

The algorithm repeatedly expands the node with the minimum accumulated cost:

```text
n* = arg min g(n)
```

until the goal is reached.

Eight-connected grid motion is used:

```text
↖  ↑  ↗
 \ | /
←  x  →
 / | \
↙  ↓  ↘
```

Straight and diagonal movements therefore have different traversal costs.

Dijkstra provides a useful baseline because it does not use heuristic information about the goal.

---

# 5. A* Path Planning

A* extends the grid-search formulation by introducing a heuristic estimate.

The evaluation function is:

```text
f(n) = g(n) + h(n)
```

where:

```text
g(n)
```

is the accumulated cost from the start to node `n`, and:

```text
h(n)
```

is the estimated remaining distance to the goal.

The current implementation uses Euclidean distance:

```text
h(n) =
sqrt(
(x_goal - x_n)^2
+
(y_goal - y_n)^2
)
```

The heuristic allows the search to prioritize nodes that are more promising with respect to the goal.

---

# 6. Dijkstra vs A*

Both algorithms operate on the same occupancy-grid representation.

The primary difference is:

```text
Dijkstra
   ↓
Uses accumulated cost only

A*
   ↓
Accumulated cost
+
Goal-directed heuristic
```

This makes it possible to compare the effect of heuristic guidance within the same environment.

---

# 7. RRT*

The project also contains a sampling-based RRT* planner.

Instead of systematically expanding grid cells, RRT* builds a search tree through sampled configurations.

The basic process is:

```text
Random Sample
      ↓
Nearest Node
      ↓
Steer Toward Sample
      ↓
Collision Check
      ↓
Add Node
      ↓
Search Nearby Nodes
      ↓
Choose Lower-Cost Parent
      ↓
Rewire
```

Random goal bias is also included so that the planner occasionally samples the goal directly.

---

# 8. RRT* Rewiring

A major distinction between RRT and RRT* is the rewiring mechanism.

When a new node is generated, nearby nodes are evaluated.

The planner attempts to minimize:

```text
J_new =
J_parent
+
distance(parent,new)
```

If a lower-cost connection exists, the parent is changed.

Nearby existing nodes can also be rewired through the new node when this decreases their path cost.

This introduces an optimization mechanism into the sampling-based planner.

---

# 9. Heading-Aware Hybrid A*-Style Planning

The fourth planner introduces vehicle heading into the search state.

Instead of representing a state only as:

```text
(x,y)
```

the planner uses:

```text
(x,y,ψ)
```

where:

```text
ψ = vehicle heading
```

The state is therefore orientation-aware.

---

# 10. Motion Primitives

The heading-aware planner expands the vehicle using a compact set of motion primitives.

Conceptually:

```text
Turn Left
    \
     \
      → Straight
     /
    /
Turn Right
```

The next state is generated approximately as:

```text
ψ(k+1) =
ψ(k)
+
ω
```

```text
x(k+1) =
x(k)
+
L cos(ψ(k+1))
```

```text
y(k+1) =
y(k)
+
L sin(ψ(k+1))
```

where:

```text
L = motion step
ω = heading-change primitive
```

A turning penalty is included in the search cost.

This discourages unnecessary heading changes.

---

# 11. Hybrid A* Implementation Scope

The implementation in this repository is intentionally lightweight.

It should be interpreted as a:

> **heading-aware Hybrid A*-style lattice planner**

rather than a production-level Hybrid A* implementation.

The current version demonstrates:

- Continuous planar motion primitives
- Heading-aware states
- Discretized heading bins
- Obstacle checking
- Goal-directed heuristic search
- Turning penalties

It does not claim to implement every component used in advanced automotive or marine Hybrid A* systems.

---

# 12. Raw Path Generation

Each planner initially produces a raw path:

```text
Planner
   ↓
Waypoint Sequence
   ↓
P = {p1,p2,...,pN}
```

Grid-based planners may generate paths containing frequent direction changes.

These paths can be collision-free while still being undesirable for direct vehicle tracking.

---

# 13. Path Smoothing

A spline-based post-processing stage is therefore included.

The raw waypoint sequence:

```text
p1 → p2 → p3 → ... → pN
```

is converted into a smoother geometric reference:

```text
Raw Path
   ↓
Spline Interpolation
   ↓
Dense Reference Points
   ↓
Smooth Path
```

The current implementation uses SciPy spline interpolation.

---

# 14. Collision Re-Checking

Path smoothing can modify the geometry of the original collision-free path.

Therefore:

```text
Collision-Free Raw Path
          ↓
      Smoothing
          ↓
New Continuous Geometry
          ↓
   Collision Re-Check
```

is required.

If the smoothed path intersects an occupied region, the framework rejects the smoothed result and uses the original path.

This prevents smoothness from being accepted at the expense of collision avoidance.

---

# 15. Path Length

Path length is calculated as:

```text
L =
Σ ||p(k+1)-p(k)||
```

Shorter paths can reduce travel distance, but path length alone is not sufficient for evaluating navigation quality.

A short path may contain:

- Sharp turns
- Low obstacle clearance
- Poor tracking characteristics

Therefore several additional metrics are considered.

---

# 16. Obstacle Clearance

The minimum clearance between the path and obstacles is estimated using a distance transform of the occupancy map.

The metric is:

```text
C_min =
min distance(path,obstacle)
```

A larger value indicates greater geometric separation from obstacles.

This provides an additional criterion beyond binary collision checking.

---

# 17. Curvature Analysis

Curvature is estimated from the generated path.

For a continuous planar curve:

```text
κ =
(x' y'' - y' x'')
/
(x'^2 + y'^2)^(3/2)
```

The framework reports:

```text
max |κ|
```

as an indicator of the sharpest turn along the path.

High curvature can indicate a trajectory that is difficult for a physical vehicle to follow.

---

# 18. Path Smoothness

A second-difference metric is used as a simple geometric smoothness measure.

For path points:

```text
p(k)
```

the second difference is approximately:

```text
Δ²p(k) =
p(k+2)
-
2p(k+1)
+
p(k)
```

The smoothness cost is calculated from these changes.

This provides a quantitative measure for comparing highly segmented and smoother paths.

---

# 19. Planning Time

Planning time is measured using:

```text
t_plan =
t_end - t_start
```

This makes it possible to evaluate the trade-off between:

```text
Path Quality
      ↕
Computational Cost
```

which becomes particularly important for replanning applications.

---

# 20. Path Tracking

Planning performance is also evaluated using a lightweight kinematic tracking simulation.

The vehicle state is represented as:

```text
x
y
ψ
```

with approximately constant forward speed.

The kinematic equations are:

```text
x_dot =
V cos(ψ)
```

```text
y_dot =
V sin(ψ)
```

```text
ψ_dot =
ω
```

where:

```text
V = vehicle speed
ω = commanded yaw rate
```

---

# 21. Pure Pursuit-Style Tracking

The tracker identifies a point ahead of the current vehicle position along the reference path.

```text
Current Vehicle Position
          ↓
Nearest Path Point
          ↓
Look-Ahead Search
          ↓
Target Point
          ↓
Desired Heading
          ↓
Yaw-Rate Command
```

The desired heading is:

```text
ψ_d =
atan2(
y_target-y,
x_target-x
)
```

The heading error is:

```text
e_ψ =
wrap(ψ_d-ψ)
```

and the yaw-rate command is bounded:

```text
ω =
sat(Kψ e_ψ)
```

---

# 22. Tracking RMSE

The tracking error is calculated using the distance between the simulated vehicle trajectory and the reference path.

A root-mean-square metric is reported:

```text
RMSE_track =
sqrt(
mean(
e_path²
)
)
```

This provides an approximate measure of practical path trackability.

---

# 23. Benchmark Metrics

Each planner can be evaluated using:

| Metric | Purpose |
|---|---|
| Path Length | Navigation efficiency |
| Planning Time | Computational performance |
| Minimum Clearance | Obstacle separation |
| Maximum Curvature | Turn severity |
| Smoothness Cost | Geometric path quality |
| Tracking RMSE | Trackability |
| Success / Failure | Planning reliability |

This enables a multi-objective comparison instead of evaluating planners using path length alone.

---

# 24. Benchmark Architecture

The intended evaluation pipeline is:

```text
Same Environment
      +
Same Start
      +
Same Goal
        ↓
 ┌──────┼───────┬─────────┐
 ↓      ↓       ↓         ↓
A*   Dijkstra  RRT*   Hybrid A*
 └──────┴───────┴─────────┘
        ↓
Same Smoothing Procedure
        ↓
Same Collision Check
        ↓
Same Metrics
        ↓
Comparative Evaluation
```

This creates a consistent basis for planner comparison.

---

# 25. Repository Structure

```text
multi_algorithm_path_planning/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── src/
│   ├── environment/
│   │   ├── __init__.py
│   │   └── grid_map.py
│   │
│   ├── planners/
│   │   ├── __init__.py
│   │   ├── grid_search.py
│   │   ├── rrt_star.py
│   │   └── hybrid_astar.py
│   │
│   ├── smoothing/
│   │   ├── __init__.py
│   │   └── spline_smoother.py
│   │
│   ├── tracking/
│   │   ├── __init__.py
│   │   └── pure_pursuit.py
│   │
│   └── evaluation/
│       ├── __init__.py
│       └── metrics.py
│
├── examples/
│   ├── compare_planners.py
│   └── tracking_demo.py
│
└── results/
```

---

# 26. Module Description

| Module | Purpose |
|---|---|
| `grid_map.py` | Occupancy-grid environment |
| `grid_search.py` | Dijkstra and A* |
| `rrt_star.py` | Sampling-based RRT* |
| `hybrid_astar.py` | Heading-aware lattice planner |
| `spline_smoother.py` | Path smoothing and collision re-check |
| `pure_pursuit.py` | Kinematic path-tracking simulation |
| `metrics.py` | Planner-performance metrics |
| `compare_planners.py` | Multi-planner benchmark |
| `tracking_demo.py` | Planning + smoothing + tracking example |

---

# 27. Installation

Clone the repository:

```bash
git clone <repository-url>
cd multi-algorithm-path-planning
```

Install dependencies:

```bash
pip install -r requirements.txt
```

Main dependencies:

```text
NumPy
SciPy
Matplotlib
```

---

# 28. Running the Planner Benchmark

Run:

```bash
python examples/compare_planners.py
```

The script compares:

```text
Dijkstra
A*
RRT*
Hybrid A*
```

within the same generic environment.

For each successful planner, the framework evaluates:

```text
Path Length
Planning Time
Minimum Clearance
Maximum Curvature
Smoothness
```

---

# 29. Running the Tracking Demonstration

Run:

```bash
python examples/tracking_demo.py
```

The demonstration performs:

```text
A* Planning
     ↓
Spline Smoothing
     ↓
Collision Verification
     ↓
Pure Pursuit-Style Tracking
     ↓
Tracking RMSE
```

and visualizes:

- Raw planned path
- Smoothed reference
- Tracked vehicle trajectory
- Obstacles

---

# 30. Recommended Result Figures

After running the simulations, representative figures can be added under:

```text
results/
├── planner_comparison.png
├── astar_path.png
├── dijkstra_path.png
├── rrt_star_path.png
├── hybrid_astar_path.png
├── raw_vs_smoothed.png
├── path_tracking.png
├── planning_time_comparison.png
├── curvature_comparison.png
└── clearance_comparison.png
```

Only figures generated from actual simulation results should be included.

---

# 31. Recommended Results Table

A final comparison can be presented as:

| Planner | Path Length | Planning Time | Min. Clearance | Max. Curvature | Tracking RMSE |
|---|---:|---:|---:|---:|---:|
| Dijkstra | measured | measured | measured | measured | measured |
| A* | measured | measured | measured | measured | measured |
| RRT* | measured | measured | measured | measured | measured |
| Hybrid A* | measured | measured | measured | measured | measured |

No assumed or fabricated performance values are required.

---

# 32. Technologies

- Python
- NumPy
- SciPy
- Matplotlib
- Graph Search
- Sampling-Based Planning
- Path Optimization
- Trajectory Smoothing
- Autonomous Navigation
- Kinematic Vehicle Modelling
- Path Tracking
- Numerical Evaluation

---

# 33. Research Areas

The project is related to:

- Autonomous Marine Vehicles
- Autonomous Surface Vehicles
- Path Planning
- Motion Planning
- Trajectory Generation
- Guidance and Navigation
- Robotic Navigation
- Sampling-Based Planning
- Optimization
- Autonomous Systems

---

# 34. Current Scope

The current implementation includes:

- 2D occupancy-grid environments
- Static obstacles
- Dijkstra
- A*
- RRT*
- Heading-aware Hybrid A*-style planning
- Spline-based smoothing
- Collision re-checking
- Obstacle-clearance evaluation
- Curvature analysis
- Smoothness evaluation
- Kinematic path tracking
- Tracking RMSE
- Planner runtime measurement

---

# 35. Current Limitations

The current public implementation does not include:

- Real electronic navigational charts
- COLREG-aware decision making
- Dynamic obstacle prediction
- Multi-vessel interaction
- Ocean-current modelling
- Wind disturbances
- Full 3-DoF or 6-DoF vessel dynamics
- Model Predictive Control tracking
- Online local replanning
- SLAM
- Real sensor data
- Hardware experiments
- Real vessel validation

Therefore, this repository should be interpreted as an **algorithmic path-planning and tracking research framework**, rather than a complete autonomous ship-navigation system.

---

# 36. Future Extensions

Several extensions can increase the fidelity of the framework.

## Dynamic Obstacles

```text
Static Planning
      ↓
Moving Obstacles
      ↓
Trajectory Prediction
      ↓
Online Replanning
```

---

## Environmental Disturbances

Future simulations could introduce:

```text
Ocean Current
+
Wind
+
Wave-Induced Disturbances
```

and evaluate their effect on path tracking.

---

## Higher-Fidelity Vessel Dynamics

The current kinematic model could be replaced with:

```text
3-DoF Marine Dynamics

Surge
Sway
Yaw
```

followed by higher-fidelity vessel models.

---

## Advanced Tracking Control

The Pure Pursuit-style tracker could be compared against:

```text
LOS Guidance
LQR
MPC
Nonlinear MPC
```

---

## Online Replanning

Dynamic navigation could use:

```text
Global Planner
      ↓
Nominal Path
      ↓
Obstacle Update
      ↓
Collision Prediction
      ↓
Replanning
      ↓
Updated Reference
```

---

## Multi-Objective Planning

Future planners could optimize a combined objective:

```text
J =
w1 × Path Length
+
w2 × Curvature
+
w3 × Obstacle Risk
+
w4 × Energy
+
w5 × Planning Time
```

This would extend the project from shortest-path planning toward trajectory-quality optimization.

---

# 37. Public Implementation Notice

This repository contains a **generic and sanitized implementation for research and educational use**.

The public implementation intentionally excludes:

- Real vessel operational parameters
- Restricted maritime data
- Proprietary navigation systems
- Operational mission logic
- Real navigation charts
- Confidential datasets
- Platform-specific control parameters

All maps, parameters, obstacles, and vehicle configurations are generic simulation examples.

---

# 38. Status

**Research-oriented simulation and benchmarking framework / active development**

The current project demonstrates the complete pipeline:

```text
Environment
     ↓
Path Planning
     ↓
Trajectory Smoothing
     ↓
Collision Verification
     ↓
Geometric Evaluation
     ↓
Path Tracking
     ↓
Performance Benchmarking
```

The primary focus is on **comparative path planning, trajectory quality, computational performance, and autonomous navigation research**.

---

# Author

**Mehmet Ateş**

Research interests:

- Autonomous Systems
- Path Planning
- Guidance, Navigation and Control
- Marine Robotics
- Autonomous Surface Vehicles
- Model Predictive Control
- State Estimation
- Reinforcement Learning
- Multi-Agent Systems
- Robotics
