# SPDX-License-Identifier: MulanPSL-2.0
"""state — amap_nav 任务状态机 (设计 4.3).

idle → planning → executing ⇄(crossing_wait) → replanning → arrived → done
     → paused / cancelled / failed
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field

IDLE = "idle"
PLANNING = "planning"
EXECUTING = "executing"
CROSSING_WAIT = "crossing_wait"
PAUSED = "paused"
REPLANNING = "replanning"
ARRIVED = "arrived"
DONE = "done"
CANCELLED = "cancelled"
FAILED = "failed"

VALID = {IDLE, PLANNING, EXECUTING, CROSSING_WAIT, PAUSED,
         REPLANNING, ARRIVED, DONE, CANCELLED, FAILED}
TERMINAL = {DONE, CANCELLED, FAILED}


@dataclass
class Task:
    task_id: str
    dest_gcj02: tuple[float, float]
    state: str = PLANNING
    detail: str = ""
    created_at: float = field(default_factory=time.time)
    history: list[tuple[float, str]] = field(default_factory=list)

    def transition(self, new_state: str, detail: str = "") -> None:
        if new_state not in VALID:
            raise ValueError(f"invalid state: {new_state}")
        self.history.append((time.time(), f"{self.state}->{new_state}: {detail}"))
        self.state = new_state
        self.detail = detail
