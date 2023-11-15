import socket
import serial
import struct
import time
import math
from threading import Thread
import _thread
from datetime import datetime

from KIRB_python_arduino import PyArduino 
from KIRB_localization import MazeLocalization

PA = PyArduino(com_port="COM7")
ML = MazeLocalization()

##### MOVE TO PYARDUINO #####

# def transmitSerial(data):
#     print(data)
    
#     # ser.write(data.encode('ascii'))
#     # ser.write(data.encode())
#     ser.write(bytes(data, 'utf-8'))        # cherry's
#     time.sleep(1)

# def receiveSerial():
#     global responses
#     global time_rx
#     responses = ser.readline().strip().decode('ascii')
#     time.sleep(0.5)    

# def transmit(data):
#     transmitSerial(data)

# ### Simulate or Run a Rover ###
# SIMULATE = False

# # ### Network Setup ###
# HOST = '127.0.0.1'  # The server's hostname or IP address
# PORT_TX = 61200     # The port used by the *CLIENT* to receive
# PORT_RX = 61201     # The port used by the *CLIENT* to send data

# COM_PORT = 'COM7'

# # Create tx and rx threads
# ser = serial.Serial(COM_PORT, 9600, timeout=0)
# Thread(target = receiveSerial, daemon = True).start()

# time.sleep(2)

###################################################################################################

class ObstacleAvoidance():

    def __init__(self):

        self.running = True

        # store sensor labels, sensors, and sensor names in lists/dicts
        # ['FRONT', 'FRONT-LEFT', 'BACK-LEFT', 'FRONT-RIGHT', 'BACK-RIGHT', 'BACK']
        self.sensor_label_list = ['u0', 'u1', 'u2', 'u3', 'u4', 'u5']
        self.sensor_label2reading_dict = {}
        for l, s in zip(self.sensor_label_list, [0 for _ in range(len(self.sensor_label_list))]):
            self.sensor_label2reading_dict[l] = s

        # initialize movement command dict
        self.mov_dict = {'LT': [' r0--90'],
                        'RT': [' r0-90'],
                        'F': [' w0-12'],
                        'B': [' w0--12'],
                        'L': [' r0--90', ' w0-12'],
                        'R': [' r0-90', ' w0-12'],}

        #initialize sensor difference and limit and preset values
        self.left_sensor_difference = 0
        self.right_sensor_difference = 0
        self.sensor_difference_limit = 0.15
        self.e_stop_limit = 1.5
        self.square_dim = 12
        self.forward_limit = 2.8
        
    # should move to arduino
    def emergency_stop(self):
        '''
        input: self
        output: None

        emergency stop algorithm comparing sensor readings and emergency stop limit

        note: a space is added before the command due to bluetooth settings
        '''

        # check if any sensor reading is less than emergency stop limit
        if self.sensor_label2reading_dict[min(self.sensor_label2reading_dict)] < self.e_stop_limit: 
            print("Emergency stop!")
            # self.running = False
            # might change code to move away from closest wall instead of stopping
            self.move(' r0-90')

    def detect_wall(self, reading):
        '''
        input: sensor reading
        output: True if there is an immediate wall, False otherwise
        '''

        if reading > ML.wall_limit:
            return False
        
        else:
            return True

    def turn(self, command):
        '''
        input: turn command
        output: None

        converts turn command into smaller movements for smoother turning
        '''

        # get direction of turn and degree
        turn_deg = int(command.split('-')[-1])

        if '--' in command:
            while turn_deg > 0:
                self.move(' r0--4')
                if self.sensor_label2reading_dict['u5']:
                    self.move(' w0--1')
                turn_deg -= 4

        elif '-' in turn_deg:
            while turn_deg > 0:
                self.move(' r0--4')
                if self.sensor_label2reading_dict['u5']:
                    self.move(' w0--1')
                turn_deg -= 4

    def travel_straight(self, command):
        '''
        input: travel straight command
        output: None

        converts travelling straight command into smaller movements for smoother travelling
        '''

        # get direction of turn and degree
        distance = int(command.split('-')[-1])

        if '--' in command:
            while distance > 0:
                self.move(' w0--1')
                distance -= 1

        elif '-' in command:
            while distance > 0:
                self.move(' w0-1')
                distance -= 1


    def localizable_square_detection(self):
        '''
        input: self
        output: True if navigated to a square that is localizable, False otherwise
        '''

        # get all sensor readings
        self.get_sensor_readings()
        front_sensor = self.sensor_label2reading_dict['u0']
        back_sensor = self.sensor_label2reading_dict['u5']
        left_sensor = self.avg_sensor_reading('L')
        right_sensor = self.avg_sensor_reading('R')
        print('front sensor: ', front_sensor)
        print('back sensor: ', back_sensor)
        print('left sensor: ', left_sensor)
        print('right sensor: ', right_sensor)

        # if wall not detected in front, return False
        if not self.detect_wall(front_sensor):
            return False
        
        # if wall detected in front
        else:

            # if back wall detected
            if self.detect_wall(back_sensor):
                self.move(' r0-90')
                return False
            
            else:
                return True
    
    def avg_sensor_reading(self, side): ### REVISE
        '''
        input: side of interest (L or R)
        output: average sensor reading
        '''

        if side == 'L':
            s1 = self.sensor_label2reading_dict['u1']
            s2 = self.sensor_label2reading_dict['u2']

        else:
            s1 = self.sensor_label2reading_dict['u3']
            s2 = self.sensor_label2reading_dict['u4']
        
        average = (s1 + s2) / 2
            
        return average
    
    def get_sensor_readings(self):
        '''
        input: self
        output: list of sensor readings in order of F, L, B, R 
            - can be used as input to localization

        note: also stores sensor readings in self.sensor_label2reading_dict 
        '''

        # get message from buffer
        message = PA.blocking_read()

        print('message: ', message)

        # split sensor readings and store in init variables
        for sensor in message.split('|')[:6]:
            try:
                label, reading = sensor.split('=')
                self.sensor_label2reading_dict[f'u{int(label)}'] = float(reading)

            except ValueError:
                break

        # print('sensor reading dict: ', self.sensor_label2reading_dict)

        # organize sensor readings in F, L, B, R order
        f = self.sensor_label2reading_dict['u0']
        l = self.avg_sensor_reading(side='L')
        b = self.sensor_label2reading_dict['u5']
        r = self.avg_sensor_reading(side='R')
        sensor_readings = [f, l, b, r]
        
        return sensor_readings
    
    def get_closest(self):
        '''
        input: self
        output: closest sensor and reading
        '''

        # list out all side sensors
        side_sensors = ['u1', 'u2', 'u3', 'u4']

        # initialize min sensor/reading
        min_sensor = None
        min_reading = math.inf

        # loop through sensors and get min sensor/reading
        for sensor in side_sensors:
            reading = self.sensor_label2reading_dict[sensor]
            if reading < min_reading:
                min_sensor = sensor
                min_reading = reading

        return min_sensor, min_reading

    def move(self, command):
        '''
        input: command
        output: None
        '''
    
        PA.write(command)

        # # may change the sleep time
        # time.sleep(0.5)

    def convert_command(self, command_loc):
        '''
        input: command from localization module
        output: command for arduino

        converts command from localization module to command for arduino including distances 
        '''

        command_ard = self.mov_dict[command_loc]

        return command_ard
    
    def sensor_diff(self):
        '''
        input: self
        output: None

        calculate sensor differences between each set of side sensors
        '''

        # find difference between left sensors
        self.left_sensor_difference = abs(self.sensor_label2reading_dict['u1'] - self.sensor_label2reading_dict['u2'])

        # find difference between right sensors
        self.right_sensor_difference = abs(self.sensor_label2reading_dict['u3'] - self.sensor_label2reading_dict['u4'])

    def parallel(self):
        '''
        input: self
        return: False if emergency stop activated, True otherwise
        '''

        print('sensor reading dict (in parallel): ', self.sensor_label2reading_dict)

        # # check if emergency stop needed
        # self.emergency_stop()
        # if self.running == False:
        #     return self.running

        self.sensor_diff()
        
        closest_sensor, _ = self.get_closest()
        print('closest sensor: ', closest_sensor)

        print('left sensor diff: ', self.left_sensor_difference)
        print('right sensor diff: ', self.right_sensor_difference)
        
        # both sides are over sensor difference limit
        if self.left_sensor_difference > self.sensor_difference_limit and self.right_sensor_difference > self.sensor_difference_limit:
            
            print("Both sides over sensor limit")

            # CONDITION #1: 45deg placement
            # TURN 4 DEG WHEN NOT ALIGNED
            # front left is closest
            if closest_sensor == 'u1':
                print('front left closest: r0-4')
                self.move(' r0-4')
        
            # back left is closest
            elif closest_sensor == 'u2':
                print('back left closest: r0--4')
                self.move(' r0--4')

            # front right is closest
            elif closest_sensor == 'u3':
                print('front right closest: r0--4')
                self.move(' r0--4')

            # back right is closest
            elif closest_sensor == 'u4':
                print('back right closest: r0-4')
                self.move(' r0-4')

        # if there is room in front, travel forward one inch
        if self.sensor_label2reading_dict['u0'] >= self.forward_limit:
            self.move(' w0-1')
        else:
            self.move(' w0--1')

            # # if no room in front, back up two inches
            # else:
                    
            #     #placeholder move back 2 inch to give more room?
            #     self.move(' w0--2')

        # else:
        #     # if there is room in front, travel forward one inch
        #     if self.sensor_label2reading_dict['u0'] >= self.forward_limit:
        #         self.move(' w0-1')


        # # if left sensor difference is less than sensor difference limit and less than right sensor difference
        # elif self.left_sensor_difference < self.right_sensor_difference and self.left_sensor_difference < self.sensor_difference_limit and self.avg_sensor_reading('L') > self.e_stop_limit:
        #     print("Right side over sensor limit")
            
        #     #if front sensot has room, follow right wall and move forward one inch
        #     if self.sensor_label2reading_dict['u0'] >= self.forward_limit:
        #         # move forward 1 inch
        #         self.move(' w0-1')

        # # if right sensor difference is less than sensor difference limit and less than left sensor difference
        # elif self.right_sensor_difference < self.left_sensor_difference and self.right_sensor_difference < self.sensor_difference_limit and self.avg_sensor_reading('R') > self.e_stop_limit:
        #     print("Left side over sensor limit")
            
        #     #if front sensot has room, follow right wall and move forward one inch
        #     if self.sensor_label2reading_dict['u0'] >= self.forward_limit:
        #         # move forward 1 inch
        #         self.move(' w0-1')

        # # left and right sensor differences are the same
        # else:
        #     #if front sensot has room, follow right wall and move forward one inch
        #     if self.sensor_label2reading_dict['u0'] >= self.forward_limit:
        #         # move forward 1 inch
        #         self.move(' w0-1')


        # # LEFT SIDE IS NOT PARALLEL
        # elif self.left_sensor_difference > self.sensor_difference_limit and self.right_sensor_difference < self.sensor_difference_limit:
        #     print("Left side not parallel...")
            
        #     #if front sensot has room, follow right wall and move forward one inch
        #     if self.sensor_label2reading_dict['u0'] >= self.forward_limit:
        #         # move forward 1 inch
        #         self.move(' w0-1')

        # # RIGHT SIDE IS NOT PARALLEL    
        # elif self.right_sensor_difference > self.sensor_difference_limit and self.left_sensor_difference < self.sensor_difference_limit:
        #     print("Right side not parallel...")
            
        #     if self.sensor_label2reading_dict['u0'] >= self.forward_limit:
        #         # move forward 1 inch
        #         self.move(' w0-1')

        # # ROVER IS PARALLEL
        # elif self.right_sensor_difference < self.sensor_difference_limit and self.left_sensor_difference < self.sensor_difference_limit:
            
        #     if self.sensor_label2reading_dict['u0'] >= self.forward_limit:
        #         # move forward 1 inch
        #         self.move(' w0-1')

    def initial_navigation(self):
        '''
        input: self
        output: None
         
        runs until initial navigation is completed
        '''

        # # request for sensor readings
        # self.move( 'ua')

        # travels through the maze purely on obstale avoidance until robot is in a localizable square
        while self.localizable_square_detection() == False:
            print('attempt to run parallel')
            self.parallel()

        print('localizable square found!')
        
    def localize_and_navigate(self, zone, location=None):
        '''
        input: zone for navigation
        output: None

        localizes robot and navigates to loading zone or drop off zone
        '''

        # get sensor readings in a list
        sensors_list = self.get_sensor_readings()
        print('sensor list 1: ', sensors_list)

        # run localization in initial square 
        _, _, command_loc = ML.initial_localize(sensors_list)

        if command_loc == ['RT']:

            command_ard = self.convert_command(command_loc[0])
                
            for command in command_ard:
                self.move(command)

            # get sensor readings in a list
            sensors_list = self.get_sensor_readings()
            print('sensor list 2: ', sensors_list)

            # run localization in initial square 
            _, _, command_loc = ML.initial_localize(sensors_list)
            # command_ard = self.convert_command(command_loc[0])
                
            # for command in command_ard:
            #     self.move(command)


        # loops until localization is fully complete
        while True:
            localized, _, command_loc = ML.localize(sensors_list)
            print('localized: ', localized)

            if localized:
                break

            command_ard = self.convert_command(command_loc[0])

            for command in command_ard:
                self.move(command)

            sensors_list = self.get_sensor_readings()

        # navigate to loading zone
        if zone == 'loading zone':
            path, movements = ML.lz_navigation()
            print('localization zone path: ', path)

            # get square, heading, and navigation command
            for square, (command_nav, heading) in zip(path[1:], movements):
                print('current square and heading: ', square, heading)
                command_ard = self.convert_command(command_nav[0])

                for command in command_ard:
                    # MAY MOVE THIS SOMEWHERE ELSE
                    self.parallel()

                    print('command (during navigation): ', command)
                    if 'w' in command:
                        self.travel_straight(command)

                    elif 'r' in command:
                        self.turn(command)

                    # MAY MOVE THIS SOMEWHERE ELSE
                    self.parallel()
                
                # give time for robot to travel in maze
                time.sleep(5)

                # run localize again
                sensors_list = self.get_sensor_readings()
                localized, current_loc, command_loc = ML.localize(sensors_list)
                if current_loc != square:
                    self.localize_and_navigate('loading_zone')

            print('reached localization zone!')

        # navigate to drop off zone
        elif zone == 'drop off zone':
            path, movements = ML.doz_navigation(location)

            # get square, heading, and navigation command
            for square, (command_nav, heading) in zip(path[1:], movements):
                print(square, heading)
                command_ard = self.convert_command(command_nav[0])

                for command in command_ard:
                    self.move(command)

                    # MAY MOVE THIS SOMEWHERE ELSE
                    self.parallel()
                
                # give time for robot to travel in maze
                time.sleep(2)

            print('reached drop off zone!')


drop_off_loc = 'A6'

OA = ObstacleAvoidance()

# navigates to a localizable square
OA.initial_navigation()

# tries to localize then navigate to wherever
OA.localize_and_navigate('loading zone')
# OA.localize_and_navigate('drop off zone', drop_off_loc)
