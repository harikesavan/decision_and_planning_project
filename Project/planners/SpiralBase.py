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

from .Structures import PathPoint, SpiralConfig

# =============================================================================
# -- Spiral Base ----------------------------------------------------------
# =============================================================================

class SpiralBase(object):
    
    def __init__(self, order):
        self.p_params_ = (order+1, 0.0)
        self.sg_ = 0
        self.error_ = np.inf
        self.spiral_config_ = SpiralConfig()
        self.start_point_ = None
        self.end_point_ = None

    # Setters
    def set_start_point(self, start : PathPoint):
        self.start_point_ = start 
    
    def set_end_point(self, end : PathPoint):
        self.end_point_ = end

    def set_spiral_config(self, spiral_config : SpiralConfig):
        self.spiral_config_ = spiral_config
    
    def set_sg(self, sg : float):
        self.sg_ = sg

    def set_error(self, error : float):
        self.error_ = error
    

    # Getters
    def get_p_params(self):
        return self.p_params_
    
    def get_sg(self):
        return self.sg_
    
    def get_error(self):
        return self.error_
    
    def get_spiral_config(self):
        return self.spiral_config_
    
    def get_start_point(self):
        return self.start_point_
    
    def get_end_point(self):
        return self.end_point_
    
    def result_sanity_check(self):
        for p in self.p_params_:
            if np.isnan(p):
                return False
        
        return self.sg_>0