# EVOLVE-BLOCK-START
"""Baseline for BasicArmInverseKinematics.

A simple one-shot IK baseline:
- one calculateInverseKinematics call per target
- clamp to joint limits
"""

from __future__ import annotations

import json
from pathlib import Path

import pybullet as p
import pybullet_data


def _load_targets(path: Path) -> dict:
    with path.open("r", encoding="utf-8-sig") as f:
        return json.load(f)


def _joint_indices(robot_id: int) -> list[int]:
    idxs: list[int] = []
    for j in range(p.getNumJoints(robot_id)):
        if p.getJointInfo(robot_id, j)[2] == p.JOINT_REVOLUTE:
            idxs.append(j)
    return idxs[:7]


def _ee_link_index(robot_id: int, ee_link_name: str) -> int:
    for j in range(p.getNumJoints(robot_id)):
        link_name = p.getJointInfo(robot_id, j)[12].decode("utf-8")
        if link_name == ee_link_name:
            return j
    raise RuntimeError(f"end-effector link not found: {ee_link_name}")


def main() -> None:
    task_root = Path(__file__).resolve().parents[1]
    cfg = _load_targets(task_root / "references" / "targets.json")
    targets = cfg["targets"]

    physics = p.connect(p.DIRECT)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        robot_urdf = str(cfg.get("robot_urdf", "franka_panda/panda.urdf"))
        ee_link_name = str(cfg.get("ee_link_name", "panda_hand"))

        robot_id = p.loadURDF(robot_urdf, useFixedBase=True)
        joint_idxs = _joint_indices(robot_id)
        ee_idx = _ee_link_index(robot_id, ee_link_name)

        lower = [float(p.getJointInfo(robot_id, j)[8]) for j in joint_idxs]
        upper = [float(p.getJointInfo(robot_id, j)[9]) for j in joint_idxs]
        joint_ranges = [hi - lo for lo, hi in zip(lower, upper)]
        rest = [0.5 * (lo + hi) for lo, hi in zip(lower, upper)]

        solutions: list[list[float]] = []
        for tgt in targets:
            target_pos = tgt["position"]
            target_quat = tgt["quaternion"]
            q = p.calculateInverseKinematics(
                robot_id,
                ee_idx,
                targetPosition=target_pos,
                targetOrientation=target_quat,
                lowerLimits=lower,
                upperLimits=upper,
                jointRanges=joint_ranges,
                restPoses=rest,
                maxNumIterations=80,
                residualThreshold=1e-6,
            )
            q7 = [float(q[i]) for i in range(7)]
            q7 = [min(max(v, lo), hi) for v, lo, hi in zip(q7, lower, upper)]
            solutions.append(q7)

        with open("submission.json", "w", encoding="utf-8") as f:
            json.dump({"solutions": solutions}, f, indent=2)

        print("Baseline submission written to submission.json")
    finally:
        p.disconnect(physics)


if __name__ == "__main__":
    main()
# EVOLVE-BLOCK-END
