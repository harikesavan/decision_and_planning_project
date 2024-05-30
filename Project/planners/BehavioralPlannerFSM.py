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

import numpy as np
import pygame

from . import PlanningParams as params
from .Structures import State, Maneuver
from . import utils

# =============================================================================
# -- Behavioral Planner  ------------------------------------------------------
# =============================================================================

class BehavioralPlannerFSM(object):

    def __init__(self, lookahead_time, lookahead_distance_min,
                 lookahead_distance_max, speed_limit, stop_threshold_speed,
                 req_stop_time, reaction_time, max_accel, stop_line_buffer=-1):
        self._lookahead_time = lookahead_time
        self._lookahead_distance_min = lookahead_distance_min
        self._lookahead_distance_max = lookahead_distance_max
        self._speed_limit = speed_limit
        self._stop_threshold_speed = stop_threshold_speed
        self._req_stop_time = req_stop_time
        self._reaction_time = reaction_time
        self._max_accel = max_accel
        self._stop_line_buffer = stop_line_buffer

        self._prev_junction_id = -1
        self._start_stop_time = None
        self._active_maneuver = Maneuver.FOLLOW_LANE
        self._goal = None

    def get_closest_waypoint_goal(self, ego_state : State, map : carla.Map, lookahead_distance : float):
        waypoint_0 = map.get_waypoint(ego_state.location)

        if (self._active_maneuver == Maneuver.DECEL_TO_STOP or self._active_maneuver == Maneuver.STOPPED):
            transform = waypoint_0.transform
            location = transform.location

            roll = transform.rotation.roll*np.pi/180
            pitch = transform.rotation.pitch*np.pi/180
            yaw = transform.rotation.yaw*np.pi/180

            rotation = carla.Rotation(pitch, yaw, roll)
            velocity = carla.Vector3D(0,0,0)
            acceleration = carla.Vector3D(0,0,0)

            return State(location, rotation, velocity, acceleration), False
        
        lookahead_waypoints = waypoint_0.next(lookahead_distance)
        n_wp = len(lookahead_waypoints)
        
        if n_wp==0:
            return State(None, None, None, None), False
        
        waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1]

        is_goal_junction = waypoint_0.is_juntion
        cur_junction_id = waypoint_0.junction_id

        if(is_goal_junction):
            if(cur_junction_id == self._prev_junction_id):
                is_goal_junction = False
            else:
                self._prev_junction_id = cur_junction_id

        transform = waypoint_0.transform
        location = transform.location

        roll = transform.rotation.roll*np.pi/180
        pitch = transform.rotation.pitch*np.pi/180
        yaw = transform.rotation.yaw*np.pi/180

        rotation = carla.Rotation(pitch, yaw, roll)
        velocity = carla.Vector3D(0,0,0)
        acceleration = carla.Vector3D(0,0,0)
        return State(location, rotation, velocity, acceleration), is_goal_junction

    def get_look_ahead_distance(self, ego_state:State):
        velocity_mag = ego_state.velocity.length()
        accel_mag = ego_state.acceleration.length()
        t = self._lookahead_time 
        # TODO-Lookahead: One way to find a reasonable lookahead distance is to find
        # the distance you will need to come to a stop while traveling at speed V and
        # using a comfortable deceleration.
        look_ahead_distance = -1 #<- calculate value
        
        if accel_mag > 0.0:
            look_ahead_distance = (velocity_mag*t) + (0.5*accel_mag*t*t)
        else:
            look_ahead_distance = velocity_mag*t
        # Cap look_ahead_distance in range [_lookahead_distance_min, _lookahead_distance_max] 
        look_ahead_distance = min(max(look_ahead_distance,self._lookahead_distance_min),self._lookahead_distance_max)

        
        return look_ahead_distance
    
    def state_transition(self, ego_state : State, goal : State, is_goal_junction : bool, tl_state : str, sim_time: int):
        # Check with the Behavior Planner to see what we are going to do and
        # where our next goal is

        goal.acceleration.x = 0
        goal.acceleration.y = 0
        goal.acceleration.z = 0

        if self._active_maneuver == Maneuver.FOLLOW_LANE:
            if is_goal_junction:
                self._active_maneuver = Maneuver.DECEL_TO_STOP

                # Let's backup a "buffer" distance behind the "STOP" point

                # TODO-goal behind the stopping point: put the goal behind the stopping
                # point (i.e the actual goal location) by "_stop_line_buffer". HINTS:
                # remember that we need to go back in the opposite direction of the
                # goal/road, i.e you should use: ang = goal.rotation.yaw + M_PI and then
                # use cosine and sine to get x and y
                #
                ang = goal.rotation.yaw + np.pi
                goal.location.x -= self._stop_line_buffer * np.cos(ang)
                goal.location.y -= self._stop_line_buffer * np.sin(ang)
            
                # TODO-goal speed at stopping point: What should be the goal speed??
                goal.velocity.x = 0.0  # <- caluclate value
                goal.velocity.y = 0.0  # <- caluclate value
                goal.velocity.z = 0.0  # <- caluclate value

            else:
                # TODO-goal speed in nominal state: What should be the goal speed now
                # that we know we are in nominal state and we can continue freely?
                # Remember that the speed is a vector
                # HINT: self._speed_limit * np.sin/cos (goal.rotation.yaw);
                goal.velocity.x = self._speed_limit * np.cos(goal.rotation.yaw) # <- caluclate value
                goal.velocity.y = self._speed_limit * np.sin(goal.rotation.yaw) # <- caluclate value
                goal.velocity.z = 0.0 # <-  caluclate value

        elif (self._active_maneuver == Maneuver.DECEL_TO_STOP):
            # TODO-maintain the same goal when in DECEL_TO_STOP state: Make sure the
            # new goal is the same as the previous goal (self._goal). That way we
            # keep/maintain the goal at the stop line.
            goal = self._goal

            # TODO: It turns out that when we teleport, the car is always at speed
            # zero. In this the case, as soon as we enter the DECEL_TO_STOP state,
            # the condition that we are <= seld._stop_threshold_speed is ALWAYS true and we
            # move straight to "STOPPED" state. To solve this issue (since we don't
            # have a motion controller yet), you should use "distance" instead of
            # speed. Make sure the distance to the stopping point is <=
            # params.P_STOP_THRESHOLD_DISTANCE. Uncomment the line used to calculate the
            # distance
            
            distance_to_stop_sign = goal.location.distance(ego_state.location)
            # TODO-use distance rather than speed: Use distance rather than speed...
            if utils.get_magnitude(ego_state.velocity) < self._stop_threshold_speed:
                # if (distance_to_stop_sign <= params.P_STOP_THRESHOLD_DISTANCE):
                # TODO-move to STOPPED state: Now that we know we are close or at the
                # stopping point we should change state to "STOPPED"

                self._active_maneuver = Maneuver.STOPPED  
                self._start_stop_time = time.time()
        
        elif (self._active_maneuver == Maneuver.STOPPED):
            # TODO-maintain the same goal when in STOPPED state: Make sure the new goal
            # is the same as the previous goal. That way we keep/maintain the goal at
            # the stop line. goal = ...;

            goal = self._goal

            stopped_secs = (time.time() - self._start_stop_time)

            if (stopped_secs >= self._req_stop_time and tl_state!="Red"):
                # TODO-move to FOLLOW_LANE state: What state do we want to move to, when
                # we are "done" at the STOPPED state?
                
                self._active_maneuver = Maneuver.FOLLOW_LANE

        self._goal = goal
        return goal


    def get_goal(self, ego_state : State, map : carla.Map):

        # Get look-ahead distance based on Ego speed
        look_ahead_distance = self.get_look_ahead_distance(ego_state)

        # Nearest waypoint on the center of a Driving Lane.
        is_goal_junction, goal_wp = self.get_closest_waypoint_goal(ego_state, map, look_ahead_distance)

        # Get Goal
        tl_state = "none"
        goal = self.state_transition(ego_state,goal_wp, is_goal_junction, tl_state)

        return goal
    
    def get_active_maneuver(self):
        return self._active_maneuver

if __name__ == "__main__":
    planner = BehavioralPlannerFSM(0,0,0,0,0,0,0,0,0)