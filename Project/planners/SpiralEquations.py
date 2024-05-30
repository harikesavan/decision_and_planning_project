# ==============================================================================
# -- Imports  ---------------------------------------------------------
# ==============================================================================
import time

import numpy as np

# ==============================================================================
# -- Functions  ---------------------------------------------------------
# ==============================================================================

def p_to_k3(sg : float, p : np.array):
    """
    coef transformation k3 indicates cubic spiral, k5 indecate quintic spiral
    """

    result = [0,0,0,0]
    result[0] = p[0] 
    result[1] = -(11.0 * p[0] - 18.0 * p[1] + 9.0 * p[2] - 2.0 * p[3]) / (2.0 * sg)
    result[2] = (18.0 * p[0] - 45.0 * p[1] + 36.0 * p[2] - 9.0 * p[3]) / (2.0 * sg * sg)
    result[3] = -(9 * p[0] - 27.0 * p[1] + 27.0 * p[2] - 9.0 * p[3]) / (2.0 * sg * sg * sg)
    return result

# kappa, theta, dkappa funcs without transformation
def kappa_func_k3(s : np.array, sg : float, p : np.array):
   a = p_to_k3(sg, p)
   return ((a[3] * s + a[2]) * s + a[1]) * s + a[0]

def dkappa_func_k3(s : np.array, sg : float, p : np.array):
   a = p_to_k3(sg, p)
   return (3 * a[3] * s + 2 * a[2]) * s + a[1]

def theta_func_k3(s : np.array, sg : float, p : np.array): 
  a = p_to_k3(sg, p)
  return (((a[3] * s / 4 + a[2] / 3) * s + a[1] / 2) * s + a[0]) * s

def partial_theta_p1_k3(s : np.array, sg : float):
  sog = s/sg
  return ((sog * 3.375 - 7.5) * sog + 4.5) * sog * s

def partial_theta_p2_k3(s : np.array, sg : float):
    sog = s/sg
    return ((6.0 - 3.375 * sog) * sog - 2.25) * sog * s

def partial_theta_sg_k3(s : np.array, sg : float, p : np.array):
  sog = s / sg

  return ((3.375 * (p[0] - 3.0 * p[1] + 3.0 * p[2] - p[3]) * sog - 
           3.0 * (2.0 * p[0] - 5.0 * p[1] + 4.0 * p[2] - p[3])) * sog + 
          0.25 * (11.0 * p[0] - 18.0 * p[1] + 9.0 * p[2] - 2.0 * p[3])) *sog * sog
