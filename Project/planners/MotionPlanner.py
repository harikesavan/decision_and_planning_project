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
import copy

from . import cost_functions
from .CubicSpiral import CubicSpiral
from . import PlanningParams as params
from .Structures import State, Maneuver, PathPoint
from .VelocityProfileGenerator import VelocityProfileGenerator
from . import utils 

# =============================================================================
# -- Motion Planner  ----------------------------------------------------------
# =============================================================================

class MotionPlanner(object):

    def __init__(self, num_paths:int, goal_offset:float, error_tolerance : float):
        self._num_paths = num_paths       # number of lateral offset paths to generate.
        self._goal_offset = goal_offset   # lateral distance between goals.
        self._error_tolerance = error_tolerance
        self._velocity_profile_generator = VelocityProfileGenerator()
        self._velocity_profile_generator.setup(params.P_TIME_GAP, params.P_MAX_ACCEL, params.P_SLOW_SPEED)
        self._cubic_spiral = CubicSpiral()
        self._best_spiral = []
        self._prev_step_count = 0
    
    def get_goal_state_in_ego_frame(self,ego_state : State, goal_state : State):
        # Let's start by making a copy of the goal state (global reference frame)
        goal_state_ego_frame = copy.deepcopy(goal_state)
        # Translate so the ego state is at the origin in the new frame.
        # This is done by subtracting the ego_state from the goal_ego_.
        goal_state_ego_frame.location.x -= ego_state.location.x
        goal_state_ego_frame.location.y -= ego_state.location.y
        goal_state_ego_frame.location.z -= ego_state.location.z

        # Rotate such that the ego state has zero heading/yaw in the new frame.
        # We are rotating by -ego_state "yaw" to ensure the ego vehicle's
        # current yaw corresponds to theta = 0 in the new local frame.

        # Recall that the general rotation matrix around the Z axix is:
        # [cos(theta) -sin(theta)
        # sin(theta)  cos(theta)]

        theta_rad = -ego_state.rotation.yaw
        cos_theta = np.cos(theta_rad)
        sin_theta = np.sin(theta_rad)

        goal_state_ego_frame.location.x =  \
            cos_theta * goal_state_ego_frame.location.x - \
            sin_theta * goal_state_ego_frame.location.y
        goal_state_ego_frame.location.y = \
            sin_theta * goal_state_ego_frame.location.x + \
            cos_theta * goal_state_ego_frame.location.y

        # Compute the goal yaw in the local frame by subtracting off the
        # current ego yaw from the goal waypoint heading/yaw.
        goal_state_ego_frame.rotation.yaw += theta_rad

        # Ego speed is the same in both coordenates
        # the Z coordinate does not get affected by the rotation.

        # Let's make sure the yaw is within [-180, 180] or [-pi, pi] so the optimizer
        # works.
        goal_state_ego_frame.rotation.yaw = utils.keep_angle_range_rad(
            goal_state_ego_frame.rotation.yaw, -np.pi, np.pi)
    
        return goal_state_ego_frame
    
    def generate_offsets_goals_ego_frame(self, ego_state : State, goal_state : State):
        # Let's transform the "main" goal (goal state) into ego reference frame
        goal_state_ego_frame = self.get_goal_state_in_ego_frame(ego_state, goal_state)

        return self.generate_offset_goals(goal_state_ego_frame)

    def generate_offset_goals_global_frame(self, goal_state : State):
        return self.generate_offset_goals(goal_state)
    
    def generate_offset_goals(self, goal_state : State):
        # Now we need to gernerate "_num_paths" goals offset from the center goal at
        # a distance "_goal_offset".
        goals_offset = []

        # the goals will be aligned on a perpendiclular line to the heading of the
        # main goal. To get a perpendicular angle, just add 90 (or PI/2) to the main
        # goal heading.

        # TODO-Perpendicular direction: ADD pi/2 to the goal yaw
        # (goal_state.rotation.yaw)
        yaw = goal_state.rotation.yaw + np.pi/2

        for i in range(self._num_paths):
            goal_offset = goal_state.copy()
            offset = (i - (int)(self._num_paths / 2)) * self._goal_offset
            
            # TODO-offset goal location: calculate the x and y position of the offset
            # goals using "offset" (calculated above) and knowing that the goals should
            # lie on a perpendicular line to the direction (yaw) of the main goal. You
            # calculated this direction above (yaw_plus_90). HINT: use
            # np.cos(yaw_plus_90) and np.sin(yaw_plus_90)

            goal_offset.location.x += offset * np.cos(yaw)
            goal_offset.location.y += offset * np.sin(yaw)
            
            if self.valid_goal(goal_state, goal_offset):
                goals_offset.append(goal_offset)

        return goals_offset
    
    def valid_goal(self, main_goal : State, offset_goal : State):
        max_offset = ((self._num_paths / 2) + 1) * self._goal_offset
        dist = main_goal.location.distance(offset_goal.location)
        return dist < max_offset
    
    def get_best_spiral_idx(self, spirals : List[List[PathPoint]], obstacles : List[State], goal_state : State):
        best_cost = np.inf
        collisions = []
        best_spiral_idx = -1

        for i in range(len(spirals)):
            cost = self.calculate_cost(spirals[i], obstacles, goal_state)

            if cost < best_cost:
                best_cost = cost
                best_spiral_idx = i
            
            if np.isinf(cost):
                collisions.append(i)
        
        if best_spiral_idx != -1:
            collisions.append(best_spiral_idx)
            return collisions
        
        return []
        
    def transform_spirals_to_global_frame(self, spirals : List[List[PathPoint]], ego_state : State):
        
        transformed_spirals = []

        for spiral in spirals:
            transformed_spiral = []
            for path_point in spiral:
                x = ego_state.location.x + \
                    path_point.x*np.cos(ego_state.rotation.yaw) - \
                    path_point.y*np.sin(ego_state.rotation.yaw)
                
                y = ego_state.location.y + \
                    path_point.x*np.sin(ego_state.rotation.yaw) + \
                    path_point.y*np.cos(ego_state.rotation.yaw)
                
                theta = path_point.theta + ego_state.rotation.yaw

                new_path_point = PathPoint(x,y,0, theta, 
                                           path_point.kappa, 
                                           path_point.s,
                                           path_point.dkappa,
                                           path_point.ddkappa)
                
                transformed_spiral.append(new_path_point)
            
            transformed_spirals.append(transformed_spiral)
        
        return transformed_spirals
    
    def generate_spirals(self, ego_state : State, goals : List[State]):
        # Since we are on Ego Frame, the start point is always at 0, 0, 0

        x = ego_state.location.x
        y = ego_state.location.y
        z = ego_state.location.z
        theta = ego_state.rotation.yaw

        start = PathPoint(x,y,z,theta, 0, 0, 0, 0)

        spirals = []

        for goal in goals:
            x = goal.location.x
            y = goal.location.y
            z = goal.location.z
            theta = goal.rotation.yaw
            s = np.sqrt((x * x) + (y * y))

            end = PathPoint(x,y,z,theta,0,s,0,0)
            if self._cubic_spiral.generate_spiral(start, end):
                
                ok, spiral = \
                    self._cubic_spiral.get_sampled_spiral(params.P_NUM_POINTS_IN_SPIRAL)
                if ok and self.valid_spiral(spiral, goal):
                    spirals.append(spiral)
        
        return spirals
    
    def valid_spiral(self, spiral : List[PathPoint], offset_goal : State):
        n = len(spiral)
        delta_x = offset_goal.location.x - spiral[n-1].x
        delta_y = offset_goal.location.y - spiral[n-1].y

        dist = np.sqrt(delta_x*delta_x + delta_y * delta_y)

        return dist < 0.1

    def calculate_cost(self, spiral : List[PathPoint], obstacles : List[State], goal : State):
        cost = 0
        cost += cost_functions.collision_circles_cost_spiral(spiral, obstacles)
        cost += cost_functions.close_to_main_goal_cost_spiral(spiral, goal)
        return cost