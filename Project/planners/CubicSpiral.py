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
from scipy.integrate import simps
from scipy.linalg import lu

from . import PlanningParams as params
from .SpiralBase import SpiralBase
from .Structures import State, Maneuver, PathPoint
from . import SpiralEquations

# =============================================================================
# -- Cubic Spiral  ----------------------------------------------------------
# =============================================================================

class CubicSpiral(SpiralBase):

    def __init__(self):
        super().__init__(3)
    
    def generate_spiral(self, start : PathPoint, end : PathPoint):
        self.set_start_point(start)
        self.set_end_point(end)

        # starting point
        x_s = start.x
        y_s = start.y
        theta_s = np.fmod(start.theta, 2*np.pi)

        if theta_s < 0:
            theta_s += 2*np.pi
        
        # end point
        x_t = end.x - x_s
        y_t = end.y - y_s

        # with transformation
        s = np.sin(theta_s)
        c = np.cos(theta_s)
        x_g = c * x_t + s * y_t
        y_g = -s * x_t + c * y_t
        theta_g = np.fmod(end.theta, 2*np.pi)-theta_s


        while theta_g < -np.pi:
            theta_g += 2*np.pi

        while theta_g > np.pi:
            theta_g -= 2*np.pi

        sg = ((theta_g * theta_g) / 5.0 + 1.0) * np.sqrt(x_g*x_g + y_g*y_g)

        p_shoot = np.asarray([start.kappa, 0, 0, end.kappa], dtype=float)

        # intermediate params
        q_g = np.asarray([x_g, y_g, theta_g]).reshape((3,1))  # goal, x(p, sg), y(p, sg), theta(p, sg)
        jacobi = np.zeros((3,3)) # Jacobian matrix for newton method

        # simpson integrations func values in Jacobian
        # integration point initialization:
        spiral_config = self.get_spiral_config()
        ds = sg / (spiral_config.simpson_size - 1);  # bandwith for integration
        # basic theta value vectors:
        theta = np.zeros(spiral_config.simpson_size)
        cos_theta = np.zeros(spiral_config.simpson_size)
        sin_theta = np.zeros(spiral_config.simpson_size)
        # partial derivatives vectors for Jacobian
        ptp_p1 = np.zeros(spiral_config.simpson_size)
        ptp_p2 = np.zeros(spiral_config.simpson_size)
        ptp_sg = np.zeros(spiral_config.simpson_size)
        sin_ptp_p1 = np.zeros(spiral_config.simpson_size)
        sin_ptp_p2 = np.zeros(spiral_config.simpson_size)
        sin_ptp_sg = np.zeros(spiral_config.simpson_size)
        cos_ptp_p1 = np.zeros(spiral_config.simpson_size)
        cos_ptp_p2 = np.zeros(spiral_config.simpson_size)
        cos_ptp_sg = np.zeros(spiral_config.simpson_size)

        # newton iteration difference (col) vectors
        delta_q = np.zeros((3,1))  # goal difference
        delta_p = np.zeros((3,1))  # parameter difference
        q_guess = np.zeros((3,1))  # q with current paramter, delta_q = q_g - q_guess
        diff = 0.0;  # absolute error for q iteration stop
        
        for iter in range(spiral_config.newton_raphson_max_iter):
            s = np.arange((spiral_config.simpson_size)).reshape(spiral_config.simpson_size,1)
            s = s * ds

            theta = SpiralEquations.theta_func_k3(s, sg, p_shoot)
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)

            ptp_p1 = SpiralEquations.partial_theta_p1_k3(s,sg)
            ptp_p2 = SpiralEquations.partial_theta_p2_k3(s,sg)
            ptp_sg = SpiralEquations.partial_theta_sg_k3(s,sg, p_shoot)

            sin_ptp_p1 = sin_theta.flatten() * ptp_p1.flatten()
            sin_ptp_p2 = sin_theta.flatten() * ptp_p2.flatten()
            sin_ptp_sg = sin_theta.flatten() * ptp_sg.flatten()

            cos_ptp_p1 = cos_theta.flatten() * ptp_p1.flatten()
            cos_ptp_p2 = cos_theta.flatten() * ptp_p2.flatten()
            cos_ptp_sg = cos_theta.flatten() * ptp_sg.flatten()

            jacobi[0,0] = -simps(sin_ptp_p1,s.flatten())
            jacobi[0,1] = -simps(sin_ptp_p2,s.flatten())
            jacobi[0,2] = cos_theta[-1] - simps(sin_ptp_sg,s.flatten())

            jacobi[1,0] = simps(cos_ptp_p1,s.flatten())
            jacobi[1,1] = simps(cos_ptp_p2,s.flatten())
            jacobi[1,2] = sin_theta[-1] + simps(cos_ptp_sg,s.flatten())

            jacobi[2,0] = ptp_p1[-1]
            jacobi[2,1] = ptp_p2[-1]
            jacobi[2,2] = ptp_sg[-1]

            q_guess[0] = simps(cos_theta.flatten(),s.flatten())
            q_guess[1] = simps(sin_theta.flatten(),s.flatten())
            q_guess[2] = theta[-1]

            delta_q = q_g - q_guess

            diff = np.abs(np.sum(delta_q))

            if diff < spiral_config.newton_raphson_tol:
                break

            # Solve the system of linear equations 
            delta_p = np.linalg.solve(jacobi, delta_q)

            # update p, sg, ds
            p_shoot[1] = p_shoot[1] + delta_p[0,0]
            p_shoot[2] = p_shoot[2] + delta_p[1,0]
            sg += delta_p[2,0]
            ds = sg / (spiral_config.simpson_size - 1)

        self.p_params_ = p_shoot
        self.set_sg(sg)
        self.set_error(diff)

        return diff < spiral_config.newton_raphson_tol \
            and self.result_sanity_check()
    
    def get_sampled_spiral(self, n:int):
        start = self.get_start_point()
        end = self.get_end_point()

        spiral_config = self.get_spiral_config()
        sg = self.get_sg()
        # initialization
        if n < 2 or self.get_error() > spiral_config.newton_raphson_tol:
            return False, []
        
        path_points = []
        ds = sg / (n-1)

        p_value = self.p_params_.copy()
        
        x = start.x
        y = start.y
        theta = start.theta
        kappa = start.kappa
        dkappa = SpiralEquations.dkappa_func_k3(0, sg, p_value)
        path_point = PathPoint(x,y,0,theta,kappa,0,dkappa,0)
        path_points.append(path_point)

        # calculate path x, y using iterative trapezoidal method
        # initialization
        s = ds*np.arange(1,n, dtype=float)
        # calculate heading kappa along the path

        thetas = SpiralEquations.theta_func_k3(s,sg,p_value)+path_points[0].theta
        kappas = SpiralEquations.kappa_func_k3(s, sg, p_value)
        dkappas = SpiralEquations.dkappa_func_k3(s,sg, p_value)

        dx = 0
        dy = 0
        
        for k in range(len(thetas)):
            ss = s[k]
            theta = thetas[k] 
            kappa = kappas[k] 
            dkappa = dkappas[k] 
            dx = (dx/(k+1))*(k) + \
                (np.cos(np.fmod(theta,2*np.pi))+\
                 np.cos(np.fmod(path_points[k].theta,2*np.pi)))/ \
                (2*(k+1))
            
            dy = (dy/(k+1))*(k) + \
                (np.sin(np.fmod(theta,2*np.pi))+\
                 np.sin(np.fmod(path_points[k].theta,2*np.pi)))/ \
                (2*(k+1))
            
            x = ss * dx + path_points[0].x
            y = ss * dy + path_points[0].y

            path_point = PathPoint(x,y,0,theta,kappa,ss,dkappa,0)
            path_points.append(path_point)

        return True, path_points

             







