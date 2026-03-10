# EVOLVE-BLOCK-START
"""Baseline for BasicArmInverseKinematics.

A simple one-shot IK baseline:
- one calculateInverseKinematics call per target
- clamp to joint limits
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
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


def _quat_angle_rad(q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> float:
    q1 = q1_xyzw / max(1e-12, float(np.linalg.norm(q1_xyzw)))
    q2 = q2_xyzw / max(1e-12, float(np.linalg.norm(q2_xyzw)))
    dot = float(np.clip(abs(np.dot(q1, q2)), -1.0, 1.0))
    return float(2.0 * np.arccos(dot))


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
        bias_seed = [0.35 * lo + 0.65 * hi for lo, hi in zip(lower, upper)]

        solutions: list[list[float]] = []
        for tgt in targets:
            target_pos = tgt["position"]
            target_quat = tgt["quaternion"]

            pos_target_np = np.array(target_pos, dtype=float)
            quat_target_np = np.array(target_quat, dtype=float)

            best_q = rest
            best_cost = float("inf")
            for seed in (rest, bias_seed):
                q7 = list(seed)
                for _ in range(2):
                    for k, joint_idx in enumerate(joint_idxs):
                        p.resetJointState(robot_id, joint_idx, q7[k])
                    q = p.calculateInverseKinematics(
                        robot_id,
                        ee_idx,
                        targetPosition=target_pos,
                        targetOrientation=target_quat,
                        lowerLimits=lower,
                        upperLimits=upper,
                        jointRanges=joint_ranges,
                        restPoses=q7,
                        maxNumIterations=220,
                        residualThreshold=1e-8,
                    )
                    q7 = [min(max(float(q[i]), lo), hi) for i, (lo, hi) in enumerate(zip(lower, upper))]

                for k, joint_idx in enumerate(joint_idxs):
                    p.resetJointState(robot_id, joint_idx, q7[k])
                ls = p.getLinkState(robot_id, ee_idx, computeForwardKinematics=True)
                pos_err = float(np.linalg.norm(np.array(ls[4], dtype=float) - pos_target_np))
                ori_err = _quat_angle_rad(np.array(ls[5], dtype=float), quat_target_np)
                cost = pos_err + 0.1 * ori_err

                if cost < best_cost:
                    best_cost = cost
                    best_q = q7

            solutions.append(best_q)

        with open("submission.json", "w", encoding="utf-8") as f:
            json.dump({"solutions": solutions}, f, indent=2)

        print("Baseline submission written to submission.json")
    finally:
        p.disconnect(physics)


if __name__ == "__main__":
    main()
# EVOLVE-BLOCK-END
