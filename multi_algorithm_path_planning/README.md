# Multi-Algorithm Path Planning and Trajectory Optimization for Autonomous Marine Vehicles

A research-oriented benchmark framework for comparing multiple path-planning
algorithms and trajectory-smoothing methods in cluttered 2D marine-like
environments.

The project focuses on algorithmic comparison rather than any specific vessel,
mission, or operational deployment.

## Implemented planners

- Dijkstra
- A*
- RRT*
- Heading-aware Hybrid A* style lattice planner

## Implemented post-processing

- B-spline-like path smoothing using cubic interpolation
- Collision re-check after smoothing
- Curvature estimation
- Path smoothness evaluation

## Tracking

A lightweight pure-pursuit style kinematic tracker is included to evaluate
whether planned/smoothed paths are practically trackable by a generic surface
vehicle model.

## Evaluation metrics

- Path length
- Planning time
- Minimum obstacle clearance
- Maximum curvature
- Smoothness cost
- Tracking RMSE
- Success / failure

## Architecture

```text
Occupancy Map
     ↓
Planner
 ┌───┼────┬───────┐
 ↓   ↓    ↓       ↓
A* Dijkstra RRT* Hybrid A*
 └───┴────┴───────┘
     ↓
Path Smoothing
     ↓
Collision / Curvature Check
     ↓
Tracking Simulation
     ↓
Benchmark Metrics
```

## Run

```bash
pip install -r requirements.txt

python examples/compare_planners.py
python examples/tracking_demo.py
```

## Scope

This repository is a generic educational/research implementation. It does not
include COLREG rules, real chart data, real vessel parameters, or operational
navigation logic.
