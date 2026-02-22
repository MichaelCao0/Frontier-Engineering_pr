from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from omegaconf import DictConfig

from frontier_eval.algorithms.base import Algorithm
from frontier_eval.tasks.base import Task


class SingleEvalAlgorithm(Algorithm):
    """
    Minimal algorithm adapter that evaluates exactly one program once.

    Useful for checking task integration without running an evolution loop.
    """

    NAME = "singleeval"

    def __init__(self, cfg: DictConfig, repo_root: Path):
        super().__init__(cfg=cfg, repo_root=repo_root)

    async def run(self, task: Task) -> None:
        output_dir = Path(str(self.cfg.run.output_dir)).resolve()
        output_dir.mkdir(parents=True, exist_ok=True)

        program_path = task.initial_program_path()
        result = task.evaluate_program(program_path)

        payload = _normalize_result(result)
        out_file = (output_dir / "singleeval_result.json").resolve()
        out_file.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

        metrics = payload.get("metrics", {})
        score = metrics.get("combined_score", metrics.get("score"))
        print(f"Task: {task.NAME}")
        print(f"Program: {program_path}")
        print(f"Score: {score}")
        print(f"Saved: {out_file}")


def _normalize_result(result: Any) -> dict[str, Any]:
    if isinstance(result, dict):
        return {"metrics": result, "artifacts": {}}

    metrics = getattr(result, "metrics", None)
    artifacts = getattr(result, "artifacts", None)
    if isinstance(metrics, dict):
        return {
            "metrics": metrics,
            "artifacts": artifacts if isinstance(artifacts, dict) else {},
        }

    return {"metrics": {"combined_score": 0.0}, "artifacts": {"raw_result": str(result)}}

