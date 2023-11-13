from KIRB_obstacle_avoidance import ObstacleAvoidance as OA
from KIRB_localization import MazeLocalization as ML
from KIRB_python_arduino import PyArduino as PA

### define generalized movement commands to feed into obstacle avoidance
# LT = r0--90
# RT = r0-90
# F = w0-12
# B = w0--12???
# L = r0--90 + w0-12
# R = r0-90 + w0-12

mov_dict = {'LT': ['r0--90'],
            'RT': ['r0-90'],
            'F': ['w0-12'],
            'B': ['w0--12'],
            'L': ['r0--90', 'w0-12'],
            'R': ['r0-90', 'w0-12'],}

# while ML.initial_localize