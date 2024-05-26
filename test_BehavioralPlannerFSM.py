import os

import glob
import json
import unittest

from planners.BehavioralPlannerFSM import BehavioralPlannerFSM
from planners.Structures import State, Maneuver
from . import TestParams as p


class TestBehavioralPlannerFSM(unittest.TestCase):
    def setUp(self) -> None:
        self.behavioral_planner = BehavioralPlannerFSM(
            p.P_LOOKAHEAD_TIME,
            p.P_LOOKAHEAD_MIN,
            p.P_LOOKAHEAD_MAX,
            p.P_SPEED_LIMIT,
            p.P_STOP_THRESHOLD_SPEED,
            p.P_REQ_STOPPED_TIME,
            p.P_REACTION_TIME,
            p.P_MAX_ACCEL,
            p.P_STOP_LINE_BUFFER
        )

    def parse_state_transition_test(self, path):
        data = None

        with open(path) as file:
            data = json.load(file)
        
        ego_state = State.from_dict(data["ego_state"])
        goal_input = State.from_dict(data["goal_input"])
        is_junction = data["is_junction"]
        tl_state = data["tl_state"]
        sim_time = data["sim_time"]
        result = State.from_dict(data["result"])
        return ego_state, goal_input, is_junction, tl_state, sim_time, result
    
    
    def test_state_transition(self):
        PATH = os.path.dirname(os.path.abspath(__file__))
        test_behaviour_in ={
            os.path.join(PATH,"test_0"):Maneuver.FOLLOW_LANE,
            os.path.join(PATH,"test_1"):Maneuver.FOLLOW_LANE,
            os.path.join(PATH,"test_2"):Maneuver.FOLLOW_LANE,
            os.path.join(PATH,"test_3"):Maneuver.FOLLOW_LANE,
            os.path.join(PATH,"test_4"):Maneuver.DECEL_TO_STOP,
        }
        test_behaviour_out ={
            os.path.join(PATH,"test_0"):Maneuver.FOLLOW_LANE,
            os.path.join(PATH,"test_1"):Maneuver.FOLLOW_LANE,
            os.path.join(PATH,"test_2"):Maneuver.FOLLOW_LANE,
            os.path.join(PATH,"test_3"):Maneuver.FOLLOW_LANE,
            os.path.join(PATH,"test_4"):Maneuver.DECEL_TO_STOP,
        }

        for test in glob.glob(f"{PATH}/test_[0-9]"):
            ego_state, goal_input, is_junction, tl_state, sim_time, goal_res = \
            self.parse_state_transition_test(os.path.join(test,"state_transition.json"))
            self.behavioral_planner._active_maneuver = test_behaviour_in[test]
            self.behavioral_planner._goal = goal_res            

            goal = self.behavioral_planner.state_transition(ego_state, goal_input, is_junction, tl_state, None)
            next_behaviour = self.behavioral_planner.get_active_maneuver()
            print(f"\nTesting {test}", end=" ")
            self.assertEqual(goal_res, goal)
            self.assertEqual(test_behaviour_out[test],next_behaviour)
            print("PASSED")
        
