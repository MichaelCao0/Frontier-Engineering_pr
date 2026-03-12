# Dynamic Obstacle Avoidance Navigation

## 1. Background

Mobile robots in warehouses, factories, and hospitals must move quickly while safely avoiding moving objects (workers, carts, AGVs). This task challenges AI agents to develop robust navigation policies that handle multiple dynamic obstacles with complex motion patterns.

## 2. Task Definition

For each fixed scenario, control a differential-drive robot from start to goal under kinematic limits while avoiding both static and dynamic obstacles.

### 2.1 Robot Motion Model

At simulation step `dt = 0.05 s`:

```
x_{k+1} = x_k + v_k * cos(theta_k) * dt
y_{k+1} = y_k + v_k * sin(theta_k) * dt
theta_{k+1} = theta_k + omega_k * dt
```

### 2.2 Inputs

`references/scenarios.json` contains **5 fixed scenarios**. Each scenario includes:

- map bounds
- static obstacles (circles/rectangles)
- **2-3 dynamic obstacles** with piecewise-linear time trajectories (crossing, reversing patterns)
- robot limits (`radius`, `v_max`, `omega_max`, `a_max`)
- start state and goal point
- max time `T_max`

## 3. Submission Format

Submit `submission.json`:

```json
{
  "scenarios": [
    {
      "id": "scene_1",
      "timestamps": [0.0, 0.2, ...],
      "controls": [[v0, w0], [v1, w1], ...]
    }
  ]
}
```

Rules:

- `timestamps` strictly increasing and start at `0.0`
- `len(controls) == len(timestamps)`
- controls must satisfy speed/turn-rate limits
- adjacent controls must satisfy acceleration limit

## 4. Constraints

A scene fails if any of the following happens:

1. collision with static or dynamic obstacles
2. robot goes out of map bounds
3. control limits violated
4. cannot reach goal before `T_max`

Goal is reached when `distance(robot, goal) <= goal_tolerance`.

## 5. Objective and Score

- Objective: minimize arrival time.
- Feasible only if all **5 scenes** succeed.
- Score (feasible case): average arrival time over 5 scenes.
- Infeasible case: `score = null`, `feasible = false`.

## 6. Evaluator Output

`verification/evaluator.py` outputs JSON:

```json
{
  "score": 13.03,
  "feasible": true,
  "details": {
    "scene_1": {"success": true, "time": 12.15},
    "scene_2": {"success": true, "time": 12.30},
    "scene_3": {"success": true, "time": 14.10},
    "scene_4": {"success": true, "time": 11.80},
    "scene_5": {"success": true, "time": 14.80}
  }
}
```

## 7. Difficulty Notes

This benchmark presents the following challenges:
- **Multiple dynamic obstacles**: 2-3 obstacles per scene requiring coordinated avoidance
- **Complex trajectories**: Obstacles move in crossing, reversing, and variable-speed patterns
- **Time-pressure navigation**: T_max constraints require efficient path planning
- **Predictive avoidance**: Simple reactive strategies may fail; agents must anticipate obstacle motion
