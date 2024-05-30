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

from enum import Enum
from dataclasses import dataclass

# ==============================================================================
# -- Structures  ---------------------------------------------------------
# ==============================================================================

class Maneuver(Enum):
    FOLLOW_LANE = 1
    FOLLOW_VEHICLE = 2
    DECEL_TO_STOP = 3
    STOPPED = 4

    def __new__(cls, value):
        member = object.__new__(cls)
        member._value_ = value
        return member
    
    def __int__(self):
        return self.value

@dataclass
class State:
    location : carla.Location
    rotation : carla.Rotation
    velocity : carla.Vector3D
    acceleration : carla.Vector3D

    def copy(self):
        x = self.location.x
        y = self.location.y
        z = self.location.z

        pitch = self.rotation.pitch
        roll = self.rotation.roll
        yaw = self.rotation.yaw

        vx = self.velocity.x
        vy = self.velocity.y
        vz = self.velocity.z

        ax = self.acceleration.x
        ay = self.acceleration.y
        az = self.acceleration.z

        location = carla.Location(x,y,z)
        rotation = carla.Rotation(pitch, yaw, roll)
        velocity = carla.Vector3D(vx, vy, vz)
        acceleration = carla.Vector3D(ax, ay, az)

        return State(location, rotation, velocity, acceleration)

    def to_dict(self):
        return {
            "x" : self.location.x,
            "y" : self.location.y,
            "z" : self.location.z,

            "pitch" : self.rotation.pitch,
            "roll" : self.rotation.roll,
            "yaw" : self.rotation.yaw,

            "vx" : self.velocity.x,
            "vy" : self.velocity.y,
            "vz" : self.velocity.z,

            "ax": self.acceleration.x,
            "ay": self.acceleration.y,
            "az": self.acceleration.z,
        }
    
    def __eq__(self, other) -> bool:
        
        for v1, v2 in zip(self.__dict__.values(), other.__dict__.values()):
            if not v1 == v2:
                return False
        return True


    def from_dict(d):
        x = d["x"] 
        y = d["y"] 
        z = d["z"] 
        pitch = d["pitch"] 
        roll = d["roll"] 
        yaw = d["yaw"] 
        vx = d["vx"] 
        vy = d["vy"] 
        vz = d["vz"] 
        ax = d["ax"] 
        ay = d["ay"] 
        az = d["az"] 

        location = carla.Location(x,y,z)
        rot = carla.Rotation(pitch,yaw,roll)
        velocity = carla.Vector3D(vx, vy, vz)
        acceleration = carla.Vector3D(ax, ay, az)

        return State(location, rot, velocity, acceleration)
    
    def get_diff(s1, s2):
        d1 = s1.to_dict()
        d2 = s2.to_dict()
        text = ""
        for (k1, v1), (k2,v2) in zip(d1.items(), d2.items()):
            if not abs(v1 - v2) < 1e-6:
                text += f"\ns1.{k1} != s2.{k2}"
                text += f"\n{v1:.5f} != {v2:.5f}"
        return text


class MPCState:
  def __init__(self, x, y, yaw, v):
      self.x = x
      self.y = y
      self.yaw = yaw
      self.v = v

@dataclass
class ManeuverParam:
    dir : int
    target_x : float
    target_speed : float
    duration : float

@dataclass
class PathPoint:
    # Coordinates
    x : float
    y : float
    z : float

    # direction on the x-y plane
    theta : float
    # curvature on the x-y planning
    kappa : float
    # accumulated distance from beginning of the path
    s : float

    # derivative of kappa w.r.t s.
    dkappa : float
    # derivative of derivative of kappa w.r.t s.
    ddkappa : float
    
    def __eq__(self, other) -> bool:
        
        for v1, v2 in zip(self.__dict__.values(), other.__dict__.values()):
            if not v1 == v2:
                return False
        return True
    
@dataclass
class TrajectoryPoint:
    # path point
    path_point : PathPoint

    # linear velocity
    v : float  # in [m/s]
    # linear acceleration
    a : float
    # relative time from beginning of the trajectory
    relative_time : float

    def __eq__(self, other) -> bool:
        
        for v1, v2 in zip(self.__dict__.values(), other.__dict__.values()):
            if not v1 == v2:
                return False
        return True

    def to_dict(self):
        return {
            "x" : self.path_point.x, 
            "y" : self.path_point.y, 
            "z" : self.path_point.z, 
            "theta" : self.path_point.theta, 
            "kappa" : self.path_point.kappa, 
            "s" : self.path_point.s, 
            "dkappa" : self.path_point.dkappa, 
            "ddkappa" : self.path_point.ddkappa, 
            "v" : self.v,
            "a" : self.a,
            "relative_time" : self.relative_time
        }
    
    def from_dict(d):
        keys1 = ["x",
                "y",
                "z",
                "theta",
                "kappa",
                "s",
                "dkappa",
                "ddkappa"]
        keys2 = ["v", "a", "relative_time"]

        path_point = {k:d[k] for k in keys1}
        t = {k:d[k] for k in keys2}
        path_point = PathPoint(**path_point)
        return TrajectoryPoint(path_point=path_point, **t)

@dataclass
class SpiralConfig:
    simpson_size : int = 9
    newton_raphson_tol : float = 0.01
    newton_raphson_max_iter : int = 20