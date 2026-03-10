# Basic Arm Inverse Kinematics

Solve inverse kinematics for a fixed set of Franka Panda end-effector targets.

## File Structure

```text
BasicArmInverseKinematics/
├── README.md
├── README_zh-CN.md
├── Task.md
├── Task_zh-CN.md
├── references/
│   └── targets.json
├── verification/
│   ├── evaluator.py
│   └── requirements.txt
└── baseline/
    ├── solution.py
    └── result_log.txt
```

## Quick Start

1. Install dependencies:

```bash
pip install -r verification/requirements.txt
```

2. Generate baseline submission:

```bash
python baseline/solution.py
```

3. Evaluate:

```bash
python verification/evaluator.py --submission submission.json
```

Output:

```json
{
  "score": 999999.987,
  "feasible": true,
  "details": {
    "success_rate": 1.0,
    "mean_position_error_m": 0.0002,
    "mean_orientation_error_deg": 0.3
  }
}
```
