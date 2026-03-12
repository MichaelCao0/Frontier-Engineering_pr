# 动态障碍避让导航（Dynamic Obstacle Avoidance Navigation）

## 1. 背景

仓储、工厂、医院中的移动机器人需要在动态环境中快速到达目标，同时保证不碰撞。本任务挑战 AI Agent 开发鲁棒的导航策略，以应对多个具有复杂运动模式的动态障碍物。

## 2. 任务定义

在每个固定场景中，控制差分轮机器人从起点到目标点，满足运动学约束，同时避让静态和动态障碍物。

### 2.1 机器人运动模型

仿真步长固定为 `dt = 0.05 s`：

```
x_{k+1} = x_k + v_k * cos(theta_k) * dt
y_{k+1} = y_k + v_k * sin(theta_k) * dt
theta_{k+1} = theta_k + omega_k * dt
```

### 2.2 输入

`references/scenarios.json` 固定提供 **5 个场景**，每个场景包含：

- 地图边界
- 静态障碍（圆形/矩形）
- **2-3 个动态障碍**，具有分段线性时间轨迹（交叉、往返等模式）
- 机器人参数（`radius`, `v_max`, `omega_max`, `a_max`）
- 起点状态与目标点
- 最大时间 `T_max`

## 3. 提交格式

提交 `submission.json`：

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

要求：

- `timestamps` 严格递增，且起点为 `0.0`
- `len(controls) == len(timestamps)`
- 控制量满足速度/角速度上限
- 相邻控制满足加速度上限

## 4. 约束

任一场景出现以下情况即失败：

1. 与静态或动态障碍碰撞
2. 机器人越界
3. 控制量违规
4. `T_max` 内未到达目标

到达判定：`distance(robot, goal) <= goal_tolerance`。

## 5. 优化目标与评分

- 目标：最小化到达时间。
- 只有 **5/5 场景**全部成功才算可行。
- 可行时：得分为 5 个场景到达时间平均值（越小越好）。
- 不可行时：`score = null`, `feasible = false`。

## 6. 评测输出

`verification/evaluator.py` 输出 JSON：

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

## 7. 难度说明

本基准测试包含以下挑战：
- **多动态障碍物**：每场景 2-3 个障碍物，需要协调避让
- **复杂轨迹**：障碍物以交叉、往返、变速等模式运动
- **时间压力导航**：T_max 约束要求高效的路径规划
- **预测式避让**：简单反应策略可能失败；Agent 需要预判障碍物运动