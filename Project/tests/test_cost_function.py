import os

import glob
import json
import unittest

import planners.cost_functions as cost_functions
from planners.Structures import PathPoint, State
from . import TestParams as p


class TestBehavioralPlannerFSM(unittest.TestCase):
    def setUp(self) -> None:
        pass

    def parse_cost_function_test(self, path):
        data = None

        with open(path) as file:
            data = json.load(file)
        
        spiral = []
        for point in data["spiral"]:
            path_point = PathPoint(**point)
            spiral.append(path_point)

        obstacles = []

        for o in data["obstacles"]:
            obstacle = State.from_dict(o)
            obstacles.append(obstacle)

        goal = State.from_dict(data["goal"])

        collision_cost = data["collision_circles_cost_spiral"]
        distance_cost = data["close_to_main_goal_cost_spiral"]

        return spiral, obstacles, goal, collision_cost, distance_cost
    
    
    def test_state_transition(self):
        PATH = os.path.dirname(os.path.abspath(__file__))

        for test in glob.glob(f"{PATH}/test_[0-9]"):
            print(f"Testing {test}")
            for cost_test in os.listdir(f"{test}/cost_log"):
                print(f"\t {cost_test}", end=" ")

                spiral, obstacles, goal, collision_cost, distance_cost = \
                self.parse_cost_function_test(f"{test}/cost_log/{cost_test}")

                collision_cost_test = cost_functions.collision_circles_cost_spiral(spiral, obstacles)
                distance_cost_test = cost_functions.close_to_main_goal_cost_spiral(spiral, goal)


                self.assertEqual(distance_cost_test, distance_cost)
                self.assertEqual(collision_cost_test,collision_cost)
                print("PASSED")
        
