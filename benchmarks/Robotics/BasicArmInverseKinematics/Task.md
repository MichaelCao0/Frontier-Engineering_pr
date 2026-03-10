# Basic Arm Inverse Kinematics

## 1. Background

Inverse kinematics (IK) is a core primitive in robot motion planning and manipulation. This task focuses on solving a fixed set of target poses reliably and accurately.

## 2. Robot and Targets

- Robot: Franka Panda (`franka_panda/panda.urdf` from `pybullet_data`)
- End-effector link: configured in `references/targets.json`
- Target set: fixed poses (`position + quaternion`) in `references/targets.json`

## 3. Submission

`submission.json`:

```json
{
  "solutions": [
    [q1, q2, q3, q4, q5, q6, q7],
    ...
  ]
}
```

Rules:

- one 7-DoF joint vector per target
- order must match `targets.json`
- values must be numeric

## 4. Success Criteria per Target

A target is successful when all conditions hold:

1. joint values within URDF limits
2. end-effector position error `<= 0.01 m`
3. end-effector orientation error `<= 5 deg`

## 5. Objective and Score

- Primary metric: `success_rate`
- Secondary metric: mean pose error
- Scalar score:

```text
score = success_rate * 1e6 - mean_pose_error
```

where `mean_pose_error = mean(position_error + 0.1 * orientation_error_rad)`.

Feasibility:

- `feasible=true` when format is valid and `success_rate > 0`
- format-invalid submissions return `feasible=false`, `score=null`

## 6. Evaluator Output

```json
{
  "score": 999999.98,
  "feasible": true,
  "details": {
    "success_count": 50,
    "target_count": 50,
    "success_rate": 1.0,
    "mean_position_error_m": 0.0003,
    "mean_orientation_error_deg": 0.2
  }
}
```
