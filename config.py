import numpy as np
import time
import matplotlib.pyplot as plt

TIMESTEP = 0.05
ROBOT_RADIUS = 0.2
NEIGHBOR_NUM = 3
SENSING_RADIUS = 2.0
HORIZONTAL_LENGTH = 20
EPSILON = 0.1
BETA = 20

VREF = 1.0
UREF = np.array([1,0,0])
DREF = 1.2
  
W_sep = 1.0
W_dir = 2.0
W_nav = 2.0
W_u = 4e-1

STARTS = np.array([[-0.5, 0.0, 5.0],
                   [-0.5, 1.5, 5.0],
                   [-1.0, 1.0, 5.0],
                   [ 0.5, 1.0, 5.0],
                   [-0.5, 3.0, 5.0]])
NUM_UAV = STARTS.shape[0]
X_GOAL = 10.

# Obstacle x, y, r
OBSTACLES = np.array([[4.0,-1.0, 0.2],
                      [6.5, 0.5, 0.2],
                      [8.0,-1.0, 0.2],
                      [8.0, 2.0, 0.2],
                      [4.0, 2.0, 0.2],
                      [6.0, 3.0, 0.2],
                      [2.0, 0.0, 0.2],
                      [9.0, 1.0, 0.2]])