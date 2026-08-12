# SPDX-License-Identifier: MulanPSL-2.0
from __future__ import annotations

import unittest

from amap_nav_service import state as st


class TestTask(unittest.TestCase):
    def test_initial_state_planning(self):
        t = st.Task(task_id="t1", dest_gcj02=(116.0, 39.0))
        self.assertEqual(t.state, st.PLANNING)

    def test_transition_records_history(self):
        t = st.Task(task_id="t1", dest_gcj02=(116.0, 39.0))
        t.transition(st.EXECUTING, "segment 0")
        self.assertEqual(t.state, st.EXECUTING)
        self.assertEqual(len(t.history), 1)

    def test_invalid_state_rejected(self):
        t = st.Task(task_id="t1", dest_gcj02=(116.0, 39.0))
        with self.assertRaises(ValueError):
            t.transition("bogus")

    def test_terminal_states(self):
        for s in (st.DONE, st.CANCELLED, st.FAILED):
            self.assertIn(s, st.TERMINAL)

    def test_terminal_rejects_further_transition(self):
        t = st.Task(task_id="t1", dest_gcj02=(116.0, 39.0))
        t.transition(st.DONE, "done")
        with self.assertRaises(ValueError):
            t.transition(st.EXECUTING, "revive")

    def test_full_lifecycle(self):
        t = st.Task(task_id="t1", dest_gcj02=(116.0, 39.0))
        t.transition(st.EXECUTING, "start")
        t.transition(st.CROSSING_WAIT, "gate")
        t.transition(st.EXECUTING, "resume")
        t.transition(st.ARRIVED, "arrived")
        t.transition(st.DONE, "done")
        self.assertEqual(t.state, st.DONE)
        self.assertEqual(len(t.history), 5)


if __name__ == "__main__":
    unittest.main()
