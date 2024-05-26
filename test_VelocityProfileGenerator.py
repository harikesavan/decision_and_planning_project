import os

import glob
import json
import unittest

from planners.VelocityProfileGenerator import VelocityProfileGenerator
from planners.Structures import State, PathPoint, Maneuver, TrajectoryPoint
from . import TestParams as p


class TestVelocityProfileGenerator(unittest.TestCase):
    def setUp(self) -> None:
        self.velocity_profile_generator = VelocityProfileGenerator()
        self.velocity_profile_generator.setup(p.P_TIME_GAP, 
                                              p.P_MAX_ACCEL, 
                                              p.P_SLOW_SPEED)

    def parse_trajectory_from_file(self, path):
        data = None

        with open(path) as file:
            data = json.load(file)
        
        spirals = []
        for s in data["spirals"]:
            spiral = []
            for point in s: 
                path_point = PathPoint(**point)
                spiral.append(path_point)
            spirals.append(spiral)
        
        trajectories = []

        for t in data["result"]:
            trajectory = []
            for point in t:
                trajec_point = TrajectoryPoint.from_dict(point)
                trajectory.append(trajec_point)
            trajectories.append(trajectory)

        desired_speed = data["desired_speed"]
        ego_state = State.from_dict(data["ego_state"])
        behaviour  = Maneuver(data["behaviour"])

        return spirals, desired_speed, ego_state, behaviour, trajectories
    
    def test_generate_trajectory(self):
        PATH = os.path.dirname(os.path.abspath(__file__))

        for test in glob.glob(f"{PATH}/test_[0-9]"):
            spirals, desired_speed, ego_state, behaviour, trajectories = \
                self.parse_trajectory_from_file(f"{test}/generate_trajectory.json")

            print(f"\nTesting {test}")
            for i, (spiral, trajectory) in enumerate(zip(spirals, trajectories)):
                print(f"\tTrajectory {i}", end = " ")
                trajectory_calculated = self.velocity_profile_generator.generate_trajectory(
                    spiral,
                    desired_speed,
                    ego_state,
                    None,
                    behaviour
                )
                self.assertListEqual(trajectory, trajectory_calculated)

                print("PASSED")