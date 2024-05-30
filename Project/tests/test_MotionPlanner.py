import os

import glob
import json
import unittest


from planners.MotionPlanner import MotionPlanner
from planners.Structures import State
from . import TestParams as p


class TestMotionPlanner(unittest.TestCase):
    def setUp(self) -> None:
        self.motion_planner = MotionPlanner(
            p.P_NUM_PATHS,
            p.P_GOAL_OFFSET,
            p.P_ERR_TOLERANCE
        )

        self.sort_state_func = lambda state : \
            (state.location.x, state.location.y, state.location.z)


    def parse_offset_test(self, path):
        data = None

        with open(path) as file:
            data = json.load(file)
        
        goal_input = State.from_dict(data["goal_input"])
        result = [State.from_dict(r) for r in data["result"]]
        return goal_input, result
    
    def test_generate_offset_goals(self):
        PATH = os.path.dirname(os.path.abspath(__file__))

        for test in glob.glob(f"{PATH}/test_[0-9]"):
            goal, goals_offset = self.parse_offset_test(f"{test}/generate_offset_goals.json")
            goal_set = self.motion_planner.generate_offset_goals(goal)

            goals_offset = sorted(goals_offset, key=self.sort_state_func)
            goal_set = sorted(goal_set, key=self.sort_state_func)
            print(f"\nTesting {test}")
            for i, (g1, g2) in enumerate(zip(goal_set, goals_offset)):
                    print(f"\Goal {i}", end = " ")
                    self.assertEqual(g1,g2)
                    print("PASSED")
