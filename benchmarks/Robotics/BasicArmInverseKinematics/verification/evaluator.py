"""Evaluator for BasicArmInverseKinematics."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import pybullet as p
import pybullet_data


def _quat_angle_rad(q1_xyzw: np.ndarray, q2_xyzw: np.ndarray) -> float:
    q1 = q1_xyzw / max(1e-12, float(np.linalg.norm(q1_xyzw)))
    q2 = q2_xyzw / max(1e-12, float(np.linalg.norm(q2_xyzw)))
    dot = float(np.clip(abs(np.dot(q1, q2)), -1.0, 1.0))
    return float(2.0 * np.arccos(dot))


def _load_targets(targets_path: Path) -> tuple[dict[str, Any], list[dict[str, Any]]] | tuple[None, None]:
    try:
        with targets_path.open("r", encoding="utf-8-sig") as f:
            cfg = json.load(f)
    except Exception:
        return None, None
    targets = cfg.get("targets")
    if not isinstance(targets, list) or len(targets) == 0:
        return None, None
    return cfg, targets


def evaluate(submission_path: Path, targets_path: Path | None = None) -> dict[str, Any]:
    task_root = Path(__file__).resolve().parents[1]
    targets_path = targets_path or (task_root / "references" / "targets.json")

    cfg, targets = _load_targets(targets_path)
    if cfg is None or targets is None:
        return {"score": None, "feasible": False, "details": {"reason": "invalid_targets_json"}}

    try:
        with submission_path.open("r", encoding="utf-8-sig") as f:
            submission = json.load(f)
    except Exception as exc:
        return {"score": None, "feasible": False, "details": {"reason": f"invalid_submission_json: {exc}"}}

    if not isinstance(submission, dict) or not isinstance(submission.get("solutions"), list):
        return {"score": None, "feasible": False, "details": {"reason": "missing_solutions_array"}}

    solutions = submission["solutions"]
    if len(solutions) != len(targets):
        return {"score": None, "feasible": False, "details": {"reason": "target_solution_length_mismatch"}}

    physics = p.connect(p.DIRECT)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        robot_urdf = str(cfg.get("robot_urdf", "franka_panda/panda.urdf"))
        ee_link_name = str(cfg.get("ee_link_name", "panda_hand"))
        pos_tol = float(cfg.get("position_tolerance_m", 0.01))
        ori_tol_deg = float(cfg.get("orientation_tolerance_deg", 5.0))
        ori_tol_rad = float(np.deg2rad(ori_tol_deg))

        robot_id = p.loadURDF(robot_urdf, useFixedBase=True)

        joint_idxs: list[int] = []
        for j in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, j)
            if info[2] == p.JOINT_REVOLUTE:
                joint_idxs.append(j)
        if len(joint_idxs) < 7:
            return {"score": None, "feasible": False, "details": {"reason": "expected_at_least_7_revolute_joints"}}
        joint_idxs = joint_idxs[:7]

        ee_idx = None
        for j in range(p.getNumJoints(robot_id)):
            info = p.getJointInfo(robot_id, j)
            link_name = info[12].decode("utf-8")
            if link_name == ee_link_name:
                ee_idx = j
                break
        if ee_idx is None:
            return {"score": None, "feasible": False, "details": {"reason": f"ee_link_not_found: {ee_link_name}"}}

        lower = np.array([p.getJointInfo(robot_id, j)[8] for j in joint_idxs], dtype=float)
        upper = np.array([p.getJointInfo(robot_id, j)[9] for j in joint_idxs], dtype=float)

        success_count = 0
        pos_errors: list[float] = []
        ori_errors_deg: list[float] = []
        pose_errors: list[float] = []

        for i, target in enumerate(targets):
            sol = solutions[i]
            if not isinstance(sol, list) or len(sol) != 7:
                return {"score": None, "feasible": False, "details": {"reason": f"invalid_solution_shape_at_{i}"}}
            try:
                q = np.array(sol, dtype=float)
            except Exception:
                return {"score": None, "feasible": False, "details": {"reason": f"non_numeric_solution_at_{i}"}}

            in_limit = bool(np.all(q >= lower - 1e-9) and np.all(q <= upper + 1e-9))
            if in_limit:
                for k, joint_idx in enumerate(joint_idxs):
                    p.resetJointState(robot_id, joint_idx, float(q[k]))
                ls = p.getLinkState(robot_id, ee_idx, computeForwardKinematics=True)
                pos_cur = np.array(ls[4], dtype=float)
                quat_cur = np.array(ls[5], dtype=float)
            else:
                pos_cur = np.full(3, 1e9, dtype=float)
                quat_cur = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

            pos_tgt = np.array(target["position"], dtype=float)
            quat_tgt = np.array(target["quaternion"], dtype=float)

            pos_err = float(np.linalg.norm(pos_cur - pos_tgt))
            ori_err_rad = _quat_angle_rad(quat_cur, quat_tgt)
            ori_err_deg = float(np.rad2deg(ori_err_rad))
            pose_err = pos_err + 0.1 * ori_err_rad

            pos_errors.append(pos_err)
            ori_errors_deg.append(ori_err_deg)
            pose_errors.append(pose_err)

            if in_limit and pos_err <= pos_tol and ori_err_rad <= ori_tol_rad:
                success_count += 1

        target_count = len(targets)
        success_rate = float(success_count / target_count)
        mean_pos = float(np.mean(pos_errors))
        mean_ori_deg = float(np.mean(ori_errors_deg))
        mean_pose = float(np.mean(pose_errors))

        feasible = success_rate > 0.0
        score = float(success_rate * 1e6 - mean_pose) if feasible else None
        details = {
            "success_count": int(success_count),
            "target_count": int(target_count),
            "success_rate": success_rate,
            "mean_position_error_m": mean_pos,
            "mean_orientation_error_deg": mean_ori_deg,
            "mean_pose_error": mean_pose,
        }
        return {"score": score, "feasible": feasible, "details": details}
    finally:
        p.disconnect(physics)


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluator for BasicArmInverseKinematics")
    parser.add_argument("--submission", default="submission.json", help="Path to submission JSON")
    parser.add_argument("--targets", default=None, help="Optional path to targets JSON")
    args = parser.parse_args()

    result = evaluate(Path(args.submission), Path(args.targets) if args.targets else None)
    print(json.dumps(result))


if __name__ == "__main__":
    main()
