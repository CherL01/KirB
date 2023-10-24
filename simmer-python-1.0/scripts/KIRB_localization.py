# '''
# This file is part of SimMeR, an educational mechatronics robotics simulator.
# Initial development funded by the University of Toronto MIE Department.
# Copyright (C) 2023  Ian G. Bennett

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.

# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
# '''

# # Basic echo client, for testing purposes
# # Code modified from examples on https://realpython.com/python-sockets/
# # and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

import socket
import struct
import time
import math
from threading import Thread
import _thread
from datetime import datetime
import numpy as np

# def transmit(data):
#     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
#         try:
#             s.connect((HOST, PORT_TX))
#             s.send(data.encode('utf-8'))
#         except (ConnectionRefusedError, ConnectionResetError):
#             print('Tx Connection was refused or reset.')
#             _thread.interrupt_main()
#         except TimeoutError:
#             print('Tx socket timed out.')
#             _thread.interrupt_main()
#         except EOFError:
#             print('\nKeyboardInterrupt triggered. Closing...')
#             _thread.interrupt_main()

# def receive():
#     global responses
#     global time_rx
#     while True:
#         with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
#             try:
#                 s2.connect((HOST, PORT_RX))
#                 response_raw = s2.recv(1024)
#                 if response_raw:
#                     responses = bytes_to_list(response_raw)
#                     time_rx = datetime.now().strftime("%H:%M:%S")
#             except (ConnectionRefusedError, ConnectionResetError):
#                 print('Rx connection was refused or reset.')
#                 _thread.interrupt_main()
#             except TimeoutError:
#                 print('Response not received from robot.')
#                 _thread.interrupt_main()

# def bytes_to_list(msg):
#     num_responses = int(len(msg)/8)
#     data = struct.unpack("%sd" % str(num_responses), msg)
#     return data


# ### Network Setup ###
# HOST = '127.0.0.1'  # The server's hostname or IP address
# PORT_TX = 61200     # The port used by the *CLIENT* to receive
# PORT_RX = 61201     # The port used by the *CLIENT* to send data

# # Received responses
# responses = [False]
# time_rx = 'Never'

# # Create tx and rx threads
# Thread(target = receive, daemon = True).start()

# # Run the sequence of commands
# RUNNING = True

# maze labels
maze_labels = [[f'{y}{x}' for x in range(1, 9)] for y in ['A', 'B', 'C', 'D']]

# maze wall configurations
wall_configs = [
    [2, 1, 3, 2, math.inf, 4, math.inf, 4],
    [1, 2, math.inf, 2, 3, 0, 3, 1],
    [3, math.inf, 4, math.inf, math.inf, 3, math.inf, 3],
    [2, 3, 1, 3, 3, 2, math.inf, 4]
]

# dict mapping maze labels with wall configurations
maze_map = {}
for l, c in zip(maze_labels, wall_configs):
    for label, value in zip(l, c):
        maze_map[label] = value 

class mazeLocalization():

    # initialize maze coordinates
    def __init__(self, mazeLabels, mazeConfigs, mazeMap):
        self.mazeLabels = mazeLabels
        self.mazeConfigs = mazeConfigs
        self.mazeMap = mazeMap

    # get potential locations of current square
    def get_location(self, front=False, left=False, back=False, right=False):
        '''
        input: wall readings (True = wall, False = no wall)
        output: list of potential locations in maze (ie. A1)
        '''
        orientations = [front, left, back, right]
        wall_config = None

        print(orientations)

        if True in orientations:
            if sum(orientations) == 1:
                wall_config = 1
            elif sum(orientations) == 2:
                indices = [i+1 for i, x in enumerate(orientations) if x == True]
                if sum(indices) % 2 == 1:
                    wall_config = 2
                else: 
                    wall_config = 3
            else:
                wall_config = 4
        else:
            wall_config = 0

        print(wall_config)
        self.current_location = [k for k, v in self.mazeMap.items() if v == wall_config]

        return self.current_location


# testing
test_wall_config1 = [False, False, True, False]

Loc = mazeLocalization(maze_labels, wall_configs, maze_map)
print(Loc.get_location(*test_wall_config1))


        

# #initialize sensors
# frontSensor = 0
# leftFrontSensor = 0
# leftBackSensor = 0
# rightFrontSensor = 0
# rightBackSensor = 0

# #initialize sensor difference and limit
# leftSensorDifference = 0
# rightSensorDifference = 0
# sensorDifferenceLimit = 0.15
# eStopLimit = 1.25

# #start by rotating 90 degrees (test)
# transmit('r0-90')

# while RUNNING:
    
#     # check front sensor
#     transmit('u0')
#     time.sleep(0.08)
#     print(f"Ultrasonic FRONT reading: {round(responses[0], 3)}")
#     frontSensor = responses[0]
    
#     if frontSensor < eStopLimit:
#         RUNNING = False
#         print("Emergency stop!")
#         transmit('xx')
#         break
    
#     # check left front sensor
#     transmit('u1')
#     time.sleep(0.08)
#     print(f"Ultrasonic FRONT-LEFT reading: {round(responses[0], 3)}")
#     leftFrontSensor = responses[0]

#     # check left back sensor
#     transmit('u2')
#     time.sleep(0.08)
#     print(f"Ultrasonic BACK-LEFT reading: {round(responses[0], 3)}")
#     leftBackSensor = responses[0]
    
#     # check right front sensor
#     transmit('u3')
#     time.sleep(0.08)
#     print(f"Ultrasonic FRONT-RIGHT reading: {round(responses[0], 3)}")
#     rightFrontSensor = responses[0]

#     # check right back sensor
#     transmit('u4')
#     time.sleep(0.08)
#     print(f"Ultrasonic BACK-RIGHT reading: {round(responses[0], 3)}")
#     rightBackSensor = responses[0]
    
#     if leftBackSensor<eStopLimit or leftFrontSensor<eStopLimit or rightBackSensor<eStopLimit or rightFrontSensor<eStopLimit:
#         RUNNING = False
#         print("Emergency stop!")
#         transmit('xx')
#         break    
    
#     # find difference between left sensors
#     leftSensorDifference = abs(leftFrontSensor-leftBackSensor)
#     # find difference between right sensors
#     rightSensorDifference = abs(rightFrontSensor-rightBackSensor)
    
#     sensorList = [leftFrontSensor, leftBackSensor, rightFrontSensor, rightBackSensor]
    
#     # ROVER IS NOT PARALLEL
#     if leftSensorDifference>sensorDifferenceLimit or rightSensorDifference>sensorDifferenceLimit:
    
#         # NEITHER SIDE IS PARALLEL (Ex: Rover is 45 degrees or entered 3-way/4-way intersection)
#         if leftSensorDifference>sensorDifferenceLimit and rightSensorDifference>sensorDifferenceLimit:
            
#             # CONDITION #1: 45deg placement
            
#             closest = sensorList.index(min(sensorList))
            
#             # TURN 4 DEG WHEN NOT ALIGNED
            
#             # front left is closest
#             if closest==0:
#                 transmit('r0-4')
#                 time.sleep(0.08)
#                 #print("")
#             # back left is closest
#             elif closest==1:
#                 transmit('r0--4')
#                 time.sleep(0.08)
#                 #print("")
#             # front right is closest
#             elif closest==2:
#                 transmit('r0--4')
#                 time.sleep(0.08)
#                 #print("")
#             # back right is closest
#             elif closest==3:
#                 transmit('r0-4')
#                 time.sleep(0.08)
#                 #print("")
            
#             # CONDITION #2: 3-way/4-way intersection (NOT AT WALL YET)
#             if frontSensor>=2.5:
#                 transmit('w0-1')
#                 time.sleep(0.08)

#             # CONDITION #3: 3-way/4-way intersection (AT THE WALL)
#             elif frontSensor<2.5:
                
#                 # 3 way intersection
#                     # if this localizing, turn toward closest side
#                     # if this is dropping block off at B site, turn towards B site specified
                    
#                 # 4 way interestion
#                     # idk yet depends on where its going
                    
#                 #placeholder move back 1 inch
#                 transmit('w0--1')
#                 time.sleep(0.08)

    
#         # LEFT SIDE IS NOT PARALLEL
#         elif leftSensorDifference > sensorDifferenceLimit and rightSensorDifference < sensorDifferenceLimit:
#             print("Left side not parallel...")
            
#             #if 
#             if frontSensor >= 2.25:
#                 # move forward 1 inch
#                 transmit('w0-1')
#                 time.sleep(0.08)
            
#             # rover is parallel, if less than 3, make a uturn
#             elif frontSensor < 2.25:
                
#                 transmit('r0--90')
#                 time.sleep(0.08)
#                 print("LEFT TURN")
        
#         # RIGHT SIDE IS NOT PARALLEL    
#         elif rightSensorDifference > sensorDifferenceLimit and leftSensorDifference < sensorDifferenceLimit:
#             print("Right side not parallel...")
            
#             if frontSensor >= 2.25:
#                 # move forward 1 inch
#                 transmit('w0-1')
#                 time.sleep(0.08)
#                 print("FORWARD ONE INCH")
            
#             # rover is parallel, if less than 3, make a uturn
#             elif frontSensor < 2.25:
                
#                 transmit('r0-90')
#                 time.sleep(0.08)
#                 print("RIGHT TURN")
        
#     # ROVER IS PARALLEL
#     elif rightSensorDifference < sensorDifferenceLimit and leftSensorDifference < sensorDifferenceLimit:
        
#         if frontSensor>=2.25:
#             # move forward 1 inch
#             transmit('w0-1')
#             time.sleep(0.08)
#             print("rOVER PARALLEL. FORWARD ONE INCH.")
#         elif frontSensor < 2.25:
#             if leftFrontSensor > rightFrontSensor:
#                 transmit('r0--90')
#                 time.sleep(0.08)
#             elif leftFrontSensor < rightFrontSensor:
#                 transmit('r0-90')
#                 time.sleep(0.08)
            
            
#     print("--------------")
    
#     time.sleep(0.1)
#     ct += 1
    
#     # if ct > 1000:
#     #     RUNNING = False
#     #     print("Sequence Complete!")
    
