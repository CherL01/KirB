import socket
import serial
import struct
import time
import math
from threading import Thread
import _thread
from datetime import datetime

##### MOVE TO PYARDUINO #####

def transmitSerial(data):
    print(data)
    
    # ser.write(data.encode('ascii'))
    # ser.write(data.encode())
    ser.write(bytes(data, 'utf-8'))        # cherry's
    time.sleep(1)

def receiveSerial():
    global responses
    global time_rx
    responses = ser.readline().strip().decode('ascii')
    time.sleep(0.5)    

def transmit(data):
    transmitSerial(data)

### Simulate or Run a Rover ###
SIMULATE = False

# ### Network Setup ###
HOST = '127.0.0.1'  # The server's hostname or IP address
PORT_TX = 61200     # The port used by the *CLIENT* to receive
PORT_RX = 61201     # The port used by the *CLIENT* to send data

COM_PORT = 'COM7'

# Create tx and rx threads
ser = serial.Serial(COM_PORT, 9600, timeout=0)
Thread(target = receiveSerial, daemon = True).start()

time.sleep(2)

###################################################################################################

class ObstacleAvoidance():

    def __init__(self):

        self.running = True

        # store sensor labels, sensors, and sensor names in lists/dicts
        self.sensor_name_list = ['FRONT', 'FRONT-LEFT', 'BACK-LEFT', 'FRONT-RIGHT', 'BACK-RIGHT', 'BACK']
        self.sensor_label_list = ['u0', 'u1', 'u2', 'u3', 'u4', 'u5']
        self.sensor_name2reading_dict = {}
        for l, s in zip(self.sensor_name_list, [None for _ in range(len(self.sensor_name_list))]):
            self.sensor_name2reading_dict[l] = s

        #initialize sensor difference and limit
        self.left_sensor_difference = 0
        self.right_sensor_difference = 0
        self.sensor_difference_limit = 0.15
        self.e_stop_limit = 1.25
        
    def send_command(self, command=' ua'):
        '''
        input: command (str)
        output: None 

        note: need to include a space before command due to bluetooth settings
        '''

        transmit(command)
        time.sleep(1.5)

        pass

    def receive_readings(self):
        '''
        input: None
        output: list of sensor readings in order of F, L, B, R 
            - can be used as input to localization

        note: also stores sensor readings in self.sensor_name2reading_dict 
        '''
        
        
        receiveSerial()
        reading = responses                             # list
        print(f"Sensor readings: {reading}")
        
        sensor_readings = [float(r.split('=')[1]) for r in reading.split('|')[1:7]]
        print(sensor_readings)
        
        for i in range(len(sensor_readings)-1):
            self.sensor_name2reading_dict[self.sensor_label_list[i]] = sensor_readings[i]

        # for i in range(len(reading)-1):
        #     self.sensor_name2reading_dict[self.sensor_label_list[i]] = reading[i]
        
        self.front_sensor = self.sensor_name2reading_dict['u0']
        self.left_front_sensor = self.sensor_name2reading_dict['u1']
        self.left_back_sensor = self.sensor_name2reading_dict['u2']
        self.right_front_sensor = self.sensor_name2reading_dict['u3']
        self.right_back_sensor = self.sensor_name2reading_dict['u4']
        
        self.sensor_list = [self.front_sensor, self.left_front_sensor, self.left_back_sensor, self.right_front_sensor, self.right_back_sensor]
        
        return self.sensor_name2reading_dict






        # # initialize sensors
        # self.front_sensor = self.sensor_name2reading_dict['u0']
        # self.left_front_sensor = self.sensor_name2reading_dict['u1']
        # self.left_back_sensor = self.sensor_name2reading_dict['u2']
        # self.right_front_sensor = self.sensor_name2reading_dict['u3']
        # self.right_back_sensor = self.sensor_name2reading_dict['u4']
        # self.back_sensor = self.sensor_name2reading_dict['u5']
        # self.sensor_list = [self.front_sensor, self.left_front_sensor, self.left_back_sensor, self.right_front_sensor, self.right_back_sensor]


    def emergency_stop(self):
        '''
        input: self
        output: None

        emergency stop algorithm comparing front sensor readings and emergency stop limit
        '''

        # check if front sensor reading is less than emergency stop limit
        print('MIN',min(self.front_sensor, self.left_front_sensor, self.left_back_sensor, self.right_front_sensor, self.right_back_sensor))
        if min(self.front_sensor, self.left_front_sensor, self.left_back_sensor, self.right_front_sensor, self.right_back_sensor) < self.e_stop_limit:
            transmitSerial('xx')
            self.running = False
            print("Emergency stop!")
        

        #### might change code to move away from closest wall instead of stopping ####

    
    def sensor_diff(self):
        '''
        input: self
        output: None

        calculate sensor differences
        '''

        # find difference between left sensors
        self.left_sensor_difference = abs(self.left_front_sensor - self.left_back_sensor)

        # find difference between right sensors
        self.right_sensor_difference = abs(self.right_front_sensor - self.right_back_sensor)

    def move(self, command=' w0-1'):
        '''
        input: movement command
        output: None

        transmits command
        '''

        transmitSerial(command)
        time.sleep(0.08)
    
    def parallel(self):
        '''
        input: self
        return: None

        parallel alignment algorithm to have one side (left/right) of the robot to align parallel with a wall
        '''

        # initial command - not registered
        # transmitSerial(' r0-90')

        # start by rotating 90 degrees (test)
        # transmitSerial(' r0-90')
        time.sleep(0.2)

        while (self.running == True): # (self.PARALLEL == False) 

            # check sensors, order: front -> front left -> back left -> front right -> back right
            for sensor in [' ua']: #[' ua',' u0', ' u1', ' u2', ' u3', ' u4']:
                # self.sensor_reading(sensor)
                self.send_command(sensor)

            print('sensor dict', self.sensor_name2reading_dict)
            # check if emergency stop needed
            self.emergency_stop()
            if (self.running == False):
                break

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
OA.parallel()