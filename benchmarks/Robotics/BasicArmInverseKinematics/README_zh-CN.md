# BasicArmInverseKinematics（基础机械臂逆运动学）

针对固定的 Franka Panda 末端目标位姿集合，求解对应关节角解。

## 文件结构

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

## 快速开始

1. 安装依赖：

```bash
pip install -r verification/requirements.txt
```

2. 生成 baseline 提交：

```bash
python baseline/solution.py
```

3. 评测：

```bash
python verification/evaluator.py --submission submission.json
```
