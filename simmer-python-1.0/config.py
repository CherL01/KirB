'''
This file stores the configuration information for the simulator.
'''
# This file is part of SimMeR, an educational mechatronics robotics simulator.
# Initial development funded by the University of Toronto MIE Department.
# Copyright (C) 2023  Ian G. Bennett
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import pygame.math as pm
from devices.motors import MotorSimple
from devices.ultrasonic import Ultrasonic
from devices.gyroscope import Gyroscope
from devices.compass import Compass
from devices.infrared import Infrared
from devices.drive import Drive

# Control Flags and Setup
rand_error = False          # Use either true random error generator (True) or repeatable error generation (False)
rand_bias = False           # Use a randomized, normally distributed set of bias values for drives (placeholder, not implemented)
bias_strength = [0.05, 1]   # How intense the random drive bias is, if enabled (placeholder, not implemented)

# Network configuration for sockets
host = '127.0.0.1'
port_rx = 61200
port_tx = 61201
timeout = 300
str_encoding = 'utf-8'

# Block information
block_position = [66, 5]        # Block starting location
block_rotation = 0              # Block rotation (deg)
block_size = 3                  # Block side length in inches

# Robot information
robot_start_position = [6, 42]  # Robot starting location (in)
robot_start_rotation = 180      # Robot starting rotation (deg)
robot_width = 200/25.4                 # Robot width in inches
robot_height = 5                # Robot height in inches
robot_outline = [               # Robot outline, relative to center position (100mm/25.4mm/inch) --> inches
                pm.Vector2(-70/25.4, -100/25.4),    #bottom right corner (down)
                pm.Vector2(-100/25.4, -70/25.4),    #bottom right corner (up)
                pm.Vector2(-100/25.4, 70/25.4),    #top right corner (down)
                pm.Vector2(-70/25.4, 100/25.4),    #top right corner (up)
                pm.Vector2(70/25.4,  100/25.4),    #top left corner (up)
                pm.Vector2(100/25.4,  70/25.4),    #top left corner (down)
                pm.Vector2(100/25.4, -70/25.4),     #bottom left corner (up)
                pm.Vector2(70/25.4, -100/25.4),     #bottom left corner (up)
                ]

# Maze definition information
wall_segment_length = 12    # Length of maze wall segments (inches)
floor_segment_length = 3    # Size of floor pattern squares (inches)
walls = [[3,3,1,1,0,2,0,2],
         [3,3,0,1,1,1,1,1],
         [1,0,2,0,0,1,0,1],
         [1,1,1,1,1,1,0,2]] # Matrix to define the maze walls
floor_seed = 5489           # Randomization seed for generating correctfloor pattern

# Graphics information
frame_rate = 60             # Target frame rate (Hz)
ppi = 8                    # Number of on-screen pixels per inch on display
border_pixels = floor_segment_length * ppi  # Size of the border surrounding the maze area

background_color = (43, 122, 120)

wall_thickness = 0.25       # Thickness to draw wall segments, in inches
wall_color = (255, 0, 0)    # Tuple with wall color in (R,G,B) format

robot_thickness = 0.25      # Thickness to draw robot perimeter, in inches
robot_color = (0, 0, 255)   # Tuple with robot perimeter color in (R,G,B) format

block_thickness = 0.25      # Thickness to draw robot perimeter, in inches
block_color = (127, 127, 0) # Tuple with robot perimeter color in (R,G,B) format



### DEVICE CONFIGURATION ###
# Motors
m0_info = {
    #right motor
    'id': 'm0',
    'position': [-87.8/25.4, 35/25.4],
    'rotation': 0,
    'visible': True
}

m1_info = {
    #left motor
    'id': 'm1',
    'position': [87.8/25.4, 35/25.4],
    'rotation': 0,
    'visible': True
}

motors = {
    'm0': MotorSimple(m0_info),
    'm1': MotorSimple(m1_info)
}

# Drives
w0_info = {
    'id': 'w0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [0, 5],
    'ang_velocity': 0,
    'motors': [motors['m0'], motors['m1']],
    'motor_direction': [1, 1],
    'bias': {'x': 0, 'y': 0, 'rotation': 0.2},
    'error': {'x': 0.02, 'y': 0.05, 'rotation': 1}
}

d0_info = {
    'id': 'd0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [6, 0],
    'ang_velocity': 0,
    'motors': [motors['m0'], motors['m1']],
    'motor_direction': [1, 1],
    'bias': {'x': 0, 'y': 0, 'rotation': 0.2},
    'error': {'x': 0.05, 'y': 0.05, 'rotation': 1}
}

r0_info = {
    'id': 'r0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [0, 0],
    'ang_velocity': 120,
    'motors': [motors['m0'], motors['m1']],
    'motor_direction': [1, -1],
    'bias': {'x': 0, 'y': 0, 'rotation': 0.01},
    'error': {'x': 0.003, 'y': 0.003, 'rotation': 0.02}
}

drives = {
    'w0': Drive(w0_info),
    'r0': Drive(r0_info)
}

# Sensors
# front sensor
u0_info = {
    'id': 'u0',
    'position': [0, 114.1/25.4],
    'height': 1,
    'rotation': 0,
    'error': 0.02,
    'outline': [
        pm.Vector2(-1.378/2, -1.18/2),              # size of ultrasonic sensor and mount
        pm.Vector2(-1.378/2, 1.18/2),
        pm.Vector2(1.378/2, 1.18/2),
        pm.Vector2(1.378/2, -1.18/2)
    ],
    'visible': True,
    'visible_measurement': True
}

#left front sensor
u1_info = {
    'id': 'u1',
    'position': [94.6/25.4, 35/25.4],             # +X moves to the right?
    'height': 1,
    'rotation': -90,
    'error': 0.02,
    'outline': [
        pm.Vector2(-1.378/2, -1.18/2),              # size of ultrasonic sensor and mount
        pm.Vector2(-1.378/2, 1.18/2),
        pm.Vector2(1.378/2, 1.18/2),
        pm.Vector2(1.378/2, -1.18/2)
    ],
    'visible': True,
    'visible_measurement': True
}

#left back sensor
u2_info = {
    'id': 'u2',
    'position': [94.6/25.4, -50/25.4],
    'height': 1,
    'rotation': -90,
    'error': 0.02,
    'outline': [
        pm.Vector2(-1.378/2, -1.18/2),              # size of ultrasonic sensor and mount
        pm.Vector2(-1.378/2, 1.18/2),
        pm.Vector2(1.378/2, 1.18/2),
        pm.Vector2(1.378/2, -1.18/2)
    ],
    'visible': True,
    'visible_measurement': True
}

#right front sensor
u3_info = {
    'id': 'u3',
    'position': [-94.6/25.4, 35/25.4],
    'height': 1,
    'rotation': 90,
    'error': 0.02,
    'outline': [
        pm.Vector2(-1.378/2, -1.18/2),              # size of ultrasonic sensor and mount
        pm.Vector2(-1.378/2, 1.18/2),
        pm.Vector2(1.378/2, 1.18/2),
        pm.Vector2(1.378/2, -1.18/2)
    ],
    'visible': True,
    'visible_measurement': True
}

#right back sensor
u4_info = {
    'id': 'u4',
    'position': [-94.6/25.4, -50/25.4],
    'height': 1,
    'rotation': 90,
    'error': 0.02,
    'outline': [
        pm.Vector2(-1.378/2, -1.18/2),              # size of ultrasonic sensor and mount
        pm.Vector2(-1.378/2, 1.18/2),
        pm.Vector2(1.378/2, 1.18/2),
        pm.Vector2(1.378/2, -1.18/2)
    ],
    'visible': True,
    'visible_measurement': True
}

#back sensor
u5_info = {
    'id': 'u5',
    'position': [0, -114.1/25.4],
    'height': 1,
    'rotation': 180,
    'error': 0.02,
    'outline': [
        pm.Vector2(-1.378/2, -1.18/2),              # size of ultrasonic sensor and mount
        pm.Vector2(-1.378/2, 1.18/2),
        pm.Vector2(1.378/2, 1.18/2),
        pm.Vector2(1.378/2, -1.18/2)
    ],
    'visible': True,
    'visible_measurement': True
}

# front bottom sensor
u6_info = {
    'id': 'u6',
    'position': [0, 114.1/25.4],
    'height': 1,
    'rotation': 0,
    'error': 0.02,
    'outline': [
        pm.Vector2(-1.378/2, -1.18/2),              # size of ultrasonic sensor and mount
        pm.Vector2(-1.378/2, 1.18/2),
        pm.Vector2(1.378/2, 1.18/2),
        pm.Vector2(1.378/2, -1.18/2)
    ],
    'visible': True,
    'visible_measurement': True
}

g0_info = {
    'id': 'u0',
    'position': [0, 0],
    'rotation': 0,
    'error': 0.02,
    'bias': 0.1,
    'visible': False
}

c0_info = {
    'id': 'c0',
    'position': [0, 0],
    'rotation': 0,
    'error': 0.02,
    'bias': 0.1,
    'visible': False
}

i0_info = {
    'id': 'i0',
    'position': [0, -1],
    'height': 1.5,
    'rotation': 0,
    'fov': 60,
    'threshold': 0.7,
    'error': 0.05,
    'bias': 0.1,
    'color': (127, 127, 127),
    'visible': True,
    'visible_measurement': True
}

sensors = {
    'u0': Ultrasonic(u0_info),  #front
    'u1': Ultrasonic(u1_info),  #front left
    'u2': Ultrasonic(u2_info),  #back left
    'u3': Ultrasonic(u3_info),  #front right
    'u4': Ultrasonic(u4_info),  #back right
    'u5': Ultrasonic(u5_info),  #back
    'u6': Ultrasonic(u6_info),  #front bottom
    #'g0': Gyroscope(g0_info),
    #'c0': Compass(c0_info),
    #'i0': Infrared(i0_info)
}



### TESTING AND DEBUG SETTINGS ###
simulate_list = ['u0','u1','u2','u3','u4','u5','u6']

