import socket
import serial
import struct
import time
import math
from threading import Thread
import _thread
from datetime import datetime
# from KIRB_localization import mazeLocalization

# def write_read(x):
#     ser.write(bytes(x, 'utf-8'))
#     time.sleep(2.5)
#     data = ser.readline().strip().decode('ascii')
#     return data

# def transmitNetwork(data):
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

# def receiveNetwork():
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
#     if SIMULATE:
#         num_responses = int(len(msg)/8)
#         data = struct.unpack("%sd" % str(num_responses), msg)
#         return data
#     else:
#         num_responses = int(len(msg)/8)
#         if num_responses:
#             unpackformat = "<" + num_responses*"f4x"
#             data = struct.unpack(unpackformat, msg)
#             return data

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
    
    # while True:
        # If responses are ascii characters, use this
        
        
        # response_raw = ser.readline().strip().decode('ascii')
        # response_raw = ser.readline()                         # debug
        # print(1, response_raw)    # debug only

        # If responses are 8 bytes (4-byte floats with 4 bytes of padding 0x00 values after), use this
        # response_raw = bytes_to_list(ser.readline())

        # If response received, save it
        # if response_raw[0]:
        #     # print(2, response_raw[0])     # debug only
        #     responses = response_raw
        #     # print(3, responses[0])    # debug only
        #     time_rx = datetime.now().strftime("%H:%M:%S")

        

def transmit(data):
    transmitSerial(data)

### Simulate or Run a Rover ###
SIMULATE = False

# ### Network Setup ###
HOST = '127.0.0.1'  # The server's hostname or IP address
PORT_TX = 61200     # The port used by the *CLIENT* to receive
PORT_RX = 61201     # The port used by the *CLIENT* to send data

COM_PORT = 'COM7'
# ser = serial.Serial(COM_PORT, 9600, timeout=0) 
# Thread(target = receiveSerial, daemon=True).start()

# Create tx and rx threads
ser = serial.Serial(COM_PORT, 9600, timeout=0)
Thread(target = receiveSerial, daemon = True).start()
    
# responses = [False]
# time_rx = 'Never'
time.sleep(2)

class ObstacleAvoidance():

    def __init__(self):

        # self.PARALLEL = False
        self.running = True

        # store sensor labels, sensors, and sensor names in lists/dicts
        self.sensor_label_list = ['u0', 'u1', 'u2', 'u3', 'u4', 'u5']
        self.sensor_dict = {}
        for l, s in zip(self.sensor_label_list, [None for i in range(5)]):
            self.sensor_dict[l] = s
        self.sensor_name_list = ['FRONT', 'FRONT-LEFT', 'BACK-LEFT', 'FRONT-RIGHT', 'BACK-RIGHT', 'BACK']
        
        # # initialize sensors
        # self.front_sensor = self.sensor_dict['u0']
        # self.left_front_sensor = self.sensor_dict['u1']
        # self.left_back_sensor = self.sensor_dict['u2']
        # self.right_front_sensor = self.sensor_dict['u3']
        # self.right_back_sensor = self.sensor_dict['u4']
        # self.back_sensor = self.sensor_dict['u5']
        # self.sensor_list = [self.front_sensor, self.left_front_sensor, self.left_back_sensor, self.right_front_sensor, self.right_back_sensor]

        #initialize sensor difference and limit
        self.left_sensor_difference = 0
        self.right_sensor_difference = 0
        self.sensor_difference_limit = 0.15
        self.e_stop_limit = 1.25

    def emergency_stop(self):
        '''
        input: self
        output: None

        emergency stop algorithm comparing front sensor readings and emergency stop limit
        '''

        # check if front sensor reading is less than emergency stop limit
        # if (self.front_sensor < self.e_stop_limit) or (self.left_front_sensor < self.e_stop_limit) or (self.left_back_sensor < self.e_stop_limit) or (self.right_front_sensor < self.e_stop_limit) or (self.right_back_sensor < self.e_stop_limit):
        print('MIN',min(self.front_sensor, self.left_front_sensor, self.left_back_sensor, self.right_front_sensor, self.right_back_sensor))
        if min(self.front_sensor, self.left_front_sensor, self.left_back_sensor, self.right_front_sensor, self.right_back_sensor) < self.e_stop_limit:
            transmitSerial('xx')
            self.running = False
            print("Emergency stop!")
        

        #### might change code to move away from closest wall instead of stopping ####

    def send_command(self, command=' ua'):
        '''
        input: sensor of interest
        output: sensor reading 
        '''

        # # get sensor name from label
        # sensor_name = self.sensor_name_list[self.sensor_label_list.index(sensor_label)]

        # get sensor reading
        # transmitSerial(f' {sensor_label}')
        # transmitSerial(' ua')
        # time.sleep(5)
        # print(f"Ultrasonic {sensor_name} reading: {round(responses[0], 3)}")
        # self.sensor_dict[sensor_label] = responses[0]
        # print('RESPONSES', responses)
        
        # need to send " ua" to get sensor readings (stuck on this part!)
        
        # ser.write(bytes(command, 'utf-8'))
        # time.sleep(2.5)
        # reading = ser.readline().strip().decode('ascii') 
        # print(reading)
        
        transmit(command)
        time.sleep(1.5)
        receiveSerial()
        reading = responses                             # list
        print(f"Sensor readings: {reading}")
        
        sensor_readings = [float(r.split('=')[1]) for r in reading.split('|')[1:7]]
        print(sensor_readings)
        
        for i in range(len(sensor_readings)-1):
            self.sensor_dict[self.sensor_label_list[i]] = sensor_readings[i]

        # for i in range(len(reading)-1):
        #     self.sensor_dict[self.sensor_label_list[i]] = reading[i]
        
        self.front_sensor = self.sensor_dict['u0']
        self.left_front_sensor = self.sensor_dict['u1']
        self.left_back_sensor = self.sensor_dict['u2']
        self.right_front_sensor = self.sensor_dict['u3']
        self.right_back_sensor = self.sensor_dict['u4']
        
        self.sensor_list = [self.front_sensor, self.left_front_sensor, self.left_back_sensor, self.right_front_sensor, self.right_back_sensor]
        
        return self.sensor_dict
    
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

            print('sensor dict', self.sensor_dict)
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