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

# ==============================================================================
# -- Imports  ---------------------------------------------------------
# ==============================================================================
import time
from typing import List

import carla
import numpy as np

from . import PlanningParams as params
from . import utils
from .Structures import Maneuver, PathPoint, State

# ==============================================================================
# -- cost_functions  ---------------------------------------------------------
# ==============================================================================

TIME_DIFF = 1e1
X_DIFF = 1e12
Y_DIFF = 1e12
EFFICIENCY = 1e3
MAX_JERK = 1e8
TOTAL_JERK = 1e7
COLLISION = np.inf
DANGER = 1e3
MAX_ACCEL = 1e8
TOTAL_ACCEL = 1e8
RIGHT_LANE_CHANGE = 1e1
VEHICLE_SIZE = [5, 2]
MIN_FOLLOW_DISTANCE = 1 * VEHICLE_SIZE[0]


def diff_cost(coeff : List[float], duration : float, goals : List[float], sigma : List[float], cost_weight : float):
    
    #Penalizes trajectories whose coordinate(and derivatives)
    #differ from the goal.
    
    cost = 0
    goals = np.asarray(goals)
    evals = utils.evaluate_f_and_N_derivatives(coeff, duration, 2)

    diff = evals - goals[:len(evals)]
    cost_values = 2.0/(1+np.exp(-diff/sigma[:len(evals)])) - 1
    cost = np.sum(cost_values)

    return cost * cost_weight


def collision_circles_cost_spiral(spiral : List[PathPoint], obstacles : List[State]):
    collision = False
    n_circles = len(params.CIRCLE_OFFSETS)

    for wp in spiral:
        if collision:
            break

        cur_x = wp.x
        cur_y = wp.y
        cur_yaw = wp.theta

        for c in range(n_circles):
            if collision:
                break
            
            # TODO-Circle placement: Where should the circles be at? The code below
            # is NOT complete. HINT: use CIRCLE_OFFSETS[c], sine and cosine to
            # calculate x and y: cur_y + CIRCLE_OFFSETS[c] * std::sin/cos(cur_yaw)
            
            circle_center_x = cur_x + params.CIRCLE_OFFSETS[c] * np.cos(cur_yaw)
            circle_center_y = cur_y + params.CIRCLE_OFFSETS[c] * np.sin(cur_yaw)

            for obstacle in obstacles:
                actor_yaw = obstacle.rotation.yaw

                for c2 in range(n_circles):
                    if collision:
                        break
                    actor_center_x = obstacle.location.x + params.CIRCLE_OFFSETS[c2]*np.cos(actor_yaw)
                    actor_center_y = obstacle.location.y + params.CIRCLE_OFFSETS[c2]*np.sin(actor_yaw)

                    # TODO-Distance from circles to obstacles/actor: How do you calculate
                    # the distance between the center of each circle and the
                    # obstacle/actor

                    dist = np.sqrt((circle_center_x - actor_center_x) ** 2 + (circle_center_y - actor_center_y) ** 2)          
                    
                    if dist < params.CIRCLE_RADII[c] + params.CIRCLE_RADII[c2]:
                        collision = True
                        break 

    result = 0
    
    if collision:
        result = np.inf

    return result
    
def close_to_main_goal_cost_spiral(spiral : List[PathPoint], main_goal : State):
    # The last point on the spiral should be used to check how close we are to
    # the Main (center) goal. That way, spirals that end closer to the lane
    # center-line, and that are collision free, will be prefered.
    n = len(spiral)

    # TODO-distance between last point on spiral and main goal: How do we
    # calculate the distance between the last point on the spiral (spiral[n-1])
    # and the main goal (main_goal.location). Use spiral[n - 1].x, spiral[n -
    # 1].y and spiral[n - 1].z.
    # Use main_goal.location.x, main_goal.location.y and main_goal.location.z
    # Ex: main_goal.location.x - spiral[n - 1].x
    delta_x = main_goal.location.x - spiral[n - 1].x 
    delta_y = main_goal.location.y - spiral[n - 1].y  
    delta_z = main_goal.location.z - spiral[n - 1].z 

    dist = np.sqrt(delta_x**2 + delta_y**2 + delta_z**2) # Use the deltas to calculate the distance

    cost = 2.0/ (1+np.exp(-dist)) - 1
    return cost