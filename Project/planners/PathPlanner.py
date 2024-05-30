# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    PATH = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(glob.glob('%s/../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        PATH,
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# ==============================================================================
# -- Imports  ---------------------------------------------------------
# ==============================================================================
import time
from typing import List

import numpy as np

from .BehavioralPlannerFSM import BehavioralPlannerFSM
from .MotionPlanner import MotionPlanner
from . import PlanningParams as p
from .Structures import State, Maneuver, PathPoint
from . import utils

# ==============================================================================
# -- Main Planner  ---------------------------------------------------------
# ==============================================================================


class PathPlanner(object):

    def __init__(self):

        self._behavioral_planner = BehavioralPlannerFSM(
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
        # Declare and initialize the Motion Planner and all its class requirements
        self._motion_planner = MotionPlanner(
            p.P_NUM_PATHS,
            p.P_GOAL_OFFSET,
            p.P_ERR_TOLERANCE
        )

        self._have_obst = False
        self._obstacles = []

    def plan(self, x_points : List[float],
             y_points : List[float],
             v_points : List[float],
             yaw : float, 
             velocity : float,
             goal : State,
             is_junction : bool,
             tl_state : str,

             spirals_x : List[List[float]],
             spirals_y : List[List[float]],
             spirals_v : List[List[float]],
             best_spirals : List[int],
             sim_time : int):
        
        x = x_points[-1]
        y = y_points[-1]

        location = carla.Location(x,y,0)
        rot = carla.Rotation()
        velo = carla.Vector3D(velocity,0,0)
        accel = carla.Vector3D()

        ego_state = State(location,rot,velo,accel)

        if(len(x_points)>1):
            x1 = x_points[len(x_points)-2]
            y1 = y_points[len(y_points)-2]
            x2 = x_points[len(x_points)-1]
            y2 = y_points[len(y_points)-1]

            ego_state.rotation.yaw = utils.angle_between_points(x1,y1,x2,y2)
            ego_state.velocity.x = v_points[len(v_points)-1]

            if velocity < 0.01:
                ego_state.rotation.yaw = yaw
        
        behaviour = self._behavioral_planner.get_active_maneuver()
        goal = self._behavioral_planner.state_transition(ego_state, goal, is_junction, tl_state, sim_time)

        if behaviour == Maneuver.STOPPED:
            max_points = 20
            point_x = x_points[len(x_points)-1]
            point_y = y_points[len(y_points)-1]

            for i in range(max_points):
                
                if len(x_points) >= max_points:
                    break

                x_points.append(point_x)
                y_points.append(point_y)
                v_points.append(0)
            
            return x_points, y_points, v_points, spirals_x, spirals_y, spirals_v, best_spirals
        
        goal_set = self._motion_planner.generate_offset_goals(goal)
        spirals = self._motion_planner.generate_spirals(ego_state, goal_set)
        
        desired_speed = utils.get_magnitude(goal.velocity)

        if len(spirals) == 0:
            #print("Error: No spirals generated ")
            return [], [], [], [], [], [], []
        
        for i in range(len(spirals)):

            trajectory = self._motion_planner._velocity_profile_generator.generate_trajectory(
                spirals[i],
                desired_speed,
                ego_state,
                None,
                behaviour
            )

            spiral_x = []
            spiral_y = []
            spiral_v = []

            for trajectory_point in trajectory:
                spiral_x.append(trajectory_point.path_point.x)
                spiral_y.append(trajectory_point.path_point.y)
                spiral_v.append(trajectory_point.v)
            
            spirals_x.append(spiral_x)
            spirals_y.append(spiral_y)
            spirals_v.append(spiral_v)

        best_spirals = self._motion_planner.get_best_spiral_idx(spirals, self._obstacles, goal)
        best_spiral_idx = -1

        if len(best_spirals) > 0:
            best_spiral_idx = best_spirals[len(best_spirals)-1]
        
        index = 0
        max_points = 20
        add_points = len(spirals_x[best_spiral_idx])

        for i in range(max_points):
            if not (len(x_points) < max_points and index < add_points):
                break

            point_x = spirals_x[best_spiral_idx][index]
            point_y = spirals_y[best_spiral_idx][index]
            v = spirals_v[best_spiral_idx][index]

            x_points.append(point_x)
            y_points.append(point_y)
            v_points.append(v)
            index += 1
        
        return x_points, y_points, v_points, spirals_x, spirals_y, spirals_v, best_spirals

    def set_obst(self, x_points : List[float],
                 y_points : List[float]):
        
        for x, y in zip(x_points, y_points):
            location = carla.Location(x, y, 0)
            rotation = carla.Rotation()
            velocity = carla.Vector3D()
            acceleration = carla.Vector3D()
            obstacle = State(location, rotation, velocity, acceleration)
            self._obstacles.append(obstacle)
        
        self._have_obst = True

    def get_active_maneuver(self):
        return self._behavioral_planner.get_active_maneuver()