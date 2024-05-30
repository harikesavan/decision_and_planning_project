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

from .Structures import PathPoint

# ==============================================================================
# -- Utils  ---------------------------------------------------------
# ==============================================================================


def path_point_distance(path_point_1 : PathPoint, path_point_2 : PathPoint):
    delta_x = path_point_1.x - path_point_2.x
    delta_y = path_point_1.y - path_point_2.y
    delta_z = path_point_1.z - path_point_2.z
    return np.sqrt(delta_x*delta_x + delta_y*delta_y + delta_z * delta_z)

def keep_angle_range_rad(angle, lower_limit, upper_limit):
    if(angle < lower_limit):
        angle += 2*np.pi
    elif (angle > upper_limit):
        angle -= 2*np.pi
    
    return angle

def evaluate_f_and_N_derivatives(coefficients : List[float], t : float, n : int):
    coefficients = np.asarray(coefficients[::-1])
    values = []
    
    for i in range(n+1):
        coef_diff = np.polyder(coefficients,i)
        value = np.polyval(coef_diff, t)
        values.append(value)

    return np.asarray(values)



def angle_between_points(x1, y1, x2, y2):
    return np.arctan2(y2-y1, x2-x1)

def get_magnitude(vector : carla.Vector3D):
    x = vector.x
    y = vector.y
    z = vector.z
    
    return np.sqrt(x*x + y*y + z*z)