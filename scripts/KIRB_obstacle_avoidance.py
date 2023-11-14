import socket
# import serial
import struct
import time
import math
from threading import Thread
import _thread
from datetime import datetime

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
        self.mov_dict = {'LT': ['r0--90'],
                        'RT': ['r0-90'],
                        'F': ['w0-12'],
                        'B': ['w0--12'],
                        'L': ['r0--90', 'w0-12'],
                        'R': ['r0-90', 'w0-12'],}

        #initialize sensor difference and limit and preset values
        self.left_sensor_difference = 0
        self.right_sensor_difference = 0
        self.sensor_difference_limit = 0.15
        self.e_stop_limit = 1.25
        self.square_dim = 12
        
    def convert_command(self, command_loc):
        '''
        input: command from localization module
        output: command for arduino

        converts command from localization module to command for arduino including distances 
        '''

        command_ard = self.mov_dict[command_loc]

        return command_ard

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

    def receive_readings(self, message):
        '''
        input: message from buffer
        output: list of sensor readings in order of F, L, B, R 
            - can be used as input to localization

        note: also stores sensor readings in self.sensor_label2reading_dict 
        '''
        
        for sensor in message.split('|')[1:7]:
            label, reading = sensor.split('=')
            self.sensor_label2reading_dict[label] = float(reading)

        # sensor_readings = [float(r.split('=')[1]) for r in message.split('|')[1:7]]

        # organize sensor readings in F, L, B, R order
        f = self.sensor_label2reading_dict['u0']
        l = self.avg_sensor_reading(side='L')
        b = self.sensor_label2reading_dict['u5']
        r = self.avg_sensor_reading(side='R')
        sensor_readings = [f, l, b, r]
        
        return sensor_readings

    # should move to arduino
    def emergency_stop(self):
        '''
        input: self
        output: stop command for arduino (' xx')

        emergency stop algorithm comparing sensor readings and emergency stop limit

        note: a space is added before the command due to bluetooth settings
        '''

        # check if any sensor reading is less than emergency stop limit
        if min(self.sensor_label2reading_dict) < self.e_stop_limit: 
            print("Emergency stop!")
            self.running = False
            # might change code to move away from closest wall instead of stopping
            command_ard = ' xx'  

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
        return: closest side for alignment (L, R, None)
        '''

        self.sensor_diff()

        if max(self.left_sensor_difference, self.right_sensor_difference) < self.sensor_difference_limit:
            return None
        
        elif self.left_sensor_difference > self.right_sensor_difference:
            return 'L'
        
        else:
            return 'R'

    def get_closest(self, side):
        '''
        input
        '''
    
    def right_parallel(self):
        '''
        input: self
        output: command for parallel alignment 
        '''

        if 

        # ROVER IS NOT PARALLEL
        # NEITHER SIDE IS PARALLEL (Ex: Rover is 45 degrees or entered 3-way/4-way intersection)
        if self.left_sensor_difference > self.sensor_difference_limit and self.right_sensor_difference > self.sensor_difference_limit:
            
                
                # CONDITION #1: 45deg placement
                closest = self.sensor_list.index(min(self.sensor_list[1:]))
                
                # TURN 4 DEG WHEN NOT ALIGNED
                # front left is closest
                if closest == 1:
                    self.move(' r0-4')
            
                # back left is closest
                elif closest == 2:
                    self.move(' r0--4')

                # front right is closest
                elif closest == 3:
                    self.move(' r0--4')

                # back right is closest
                elif closest == 4:
                    self.move(' r0-4')

                
                # CONDITION #2: 3-way/4-way intersection (NOT AT WALL YET)
                if self.front_sensor >= 2.5:
                    self.move(' w0-1')

                # CONDITION #3: 3-way/4-way intersection (AT THE WALL)
                elif self.front_sensor < 2.5:
                    
                    # 3 way intersection
                        # if this localizing, turn toward closest side
                        # if this is dropping block off at B site, turn towards B site specified
                        
                    # 4 way interestion
                        # idk yet depends on where its going
                        
                    #placeholder move back 1 inch
                    self.move(' w0--1')

        
            # LEFT SIDE IS NOT PARALLEL
            elif self.left_sensor_difference > self.sensor_difference_limit and self.right_sensor_difference < self.sensor_difference_limit:
                print("Left side not parallel...")
                
                #if 
                if self.front_sensor >= 2.25:
                    # move forward 1 inch
                    self.move(' w0-1')
                
                # rover is parallel, if less than 3, make a uturn
                elif self.front_sensor < 2.25:
                    
                    self.move(' r0--90')
                    print("LEFT TURN")
            
            # RIGHT SIDE IS NOT PARALLEL    
            elif self.right_sensor_difference > self.sensor_difference_limit and self.left_sensor_difference < self.sensor_difference_limit:
                print("Right side not parallel...")
                
                if self.front_sensor >= 2.25:
                    # move forward 1 inch
                    self.move(' w0-1')
                    print("FORWARD ONE INCH")
                
                # rover is parallel, if less than 3, make a uturn
                elif self.front_sensor < 2.25:
                    
                    self.move(' r0-90')
                    print("RIGHT TURN")







        # while (self.running == True): # (self.PARALLEL == False) 

        #     # check sensors, order: front -> front left -> back left -> front right -> back right
        #     for sensor in [' ua']: #[' ua',' u0', ' u1', ' u2', ' u3', ' u4']:
        #         # self.sensor_reading(sensor)
        #         self.send_command(sensor)

        #     print('sensor dict', self.sensor_label2reading_dict)
        #     # check if emergency stop needed
        #     self.emergency_stop()
        #     if (self.running == False):
        #         break

            # ROVER IS NOT PARALLEL
            self.sensor_diff()
            if max(self.left_sensor_difference, self.right_sensor_difference) > self.sensor_difference_limit:
            
                # NEITHER SIDE IS PARALLEL (Ex: Rover is 45 degrees or entered 3-way/4-way intersection)
                if self.left_sensor_difference > self.sensor_difference_limit and self.right_sensor_difference > self.sensor_difference_limit:
                    
                    # CONDITION #1: 45deg placement
                    closest = self.sensor_list.index(min(self.sensor_list[1:]))
                    
                    # TURN 4 DEG WHEN NOT ALIGNED
                    # front left is closest
                    if closest == 1:
                        self.move(' r0-4')
                
                    # back left is closest
                    elif closest == 2:
                        self.move(' r0--4')

                    # front right is closest
                    elif closest == 3:
                        self.move(' r0--4')

                    # back right is closest
                    elif closest == 4:
                        self.move(' r0-4')

                    
                    # CONDITION #2: 3-way/4-way intersection (NOT AT WALL YET)
                    if self.front_sensor >= 2.5:
                        self.move(' w0-1')

                    # CONDITION #3: 3-way/4-way intersection (AT THE WALL)
                    elif self.front_sensor < 2.5:
                        
                        # 3 way intersection
                            # if this localizing, turn toward closest side
                            # if this is dropping block off at B site, turn towards B site specified
                            
                        # 4 way interestion
                            # idk yet depends on where its going
                            
                        #placeholder move back 1 inch
                        self.move(' w0--1')

            
                # LEFT SIDE IS NOT PARALLEL
                elif self.left_sensor_difference > self.sensor_difference_limit and self.right_sensor_difference < self.sensor_difference_limit:
                    print("Left side not parallel...")
                    
                    #if 
                    if self.front_sensor >= 2.25:
                        # move forward 1 inch
                        self.move(' w0-1')
                    
                    # rover is parallel, if less than 3, make a uturn
                    elif self.front_sensor < 2.25:
                        
                        self.move(' r0--90')
                        print("LEFT TURN")
                
                # RIGHT SIDE IS NOT PARALLEL    
                elif self.right_sensor_difference > self.sensor_difference_limit and self.left_sensor_difference < self.sensor_difference_limit:
                    print("Right side not parallel...")
                    
                    if self.front_sensor >= 2.25:
                        # move forward 1 inch
                        self.move(' w0-1')
                        print("FORWARD ONE INCH")
                    
                    # rover is parallel, if less than 3, make a uturn
                    elif self.front_sensor < 2.25:
                        
                        self.move(' r0-90')
                        print("RIGHT TURN")
                
            # ROVER IS PARALLEL
            elif self.right_sensor_difference < self.sensor_difference_limit and self.left_sensor_difference < self.sensor_difference_limit:
                
                if self.front_sensor >= 2.25:
                    # move forward 1 inch
                    self.move(' w0-1')
                    print("rOVER PARALLEL. FORWARD ONE INCH.")

                elif self.front_sensor < 2.25:
                    if self.left_front_sensor > self.right_front_sensor:
                        self.move(' r0--90')

                    elif self.left_front_sensor < self.right_front_sensor:
                        self.move(' r0-90')
                    
            print("--------------")
            
            time.sleep(0.1)











# # Received responses
# responses = [False]
# time_rx = 'Never'

# # Create tx and rx threads
# Thread(target = receive, daemon = True).start()

OA = ObstacleAvoidance()
print(OA.left_sensor_difference)
OA.sensor_label2reading_dict['u1'] = 5
OA.sensor_diff()
print(OA.left_sensor_difference)