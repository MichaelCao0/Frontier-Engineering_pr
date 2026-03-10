# BasicArmInverseKinematics

## 1. 背景

逆运动学（IK）是机械臂规划与控制的基础能力。本任务要求对固定目标位姿集输出稳定、准确的关节解。

## 2. 机器人与目标

- 机器人：Franka Panda（`pybullet_data` 内 `franka_panda/panda.urdf`）
- 末端执行器链节：由 `references/targets.json` 指定
- 目标集合：`references/targets.json` 中固定给定

## 3. 提交格式

`submission.json`：

```json
{
  "solutions": [
    [q1, q2, q3, q4, q5, q6, q7],
    ...
  ]
}
```

要求：

- 每个目标对应一个 7 维关节角解
- 顺序必须与 `targets.json` 一致
- 全部为数值类型

## 4. 单目标成功条件

1. 关节角在 URDF 限位内
2. 位置误差 `<= 0.01 m`
3. 姿态误差 `<= 5 deg`

## 5. 评分规则

- 主指标：`success_rate`
- 次指标：平均位姿误差
- 单值分数：

```text
score = success_rate * 1e6 - mean_pose_error
```

其中 `mean_pose_error = mean(position_error + 0.1 * orientation_error_rad)`。

可行性：

- 格式合法且 `success_rate > 0` 时 `feasible=true`
- 格式非法时 `feasible=false, score=null`
