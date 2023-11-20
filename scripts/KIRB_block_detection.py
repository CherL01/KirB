import math
import time

from KIRB_python_arduino import PyArduino

PA = PyArduino(com_port="COM7")

class BlockDetection():

    # initialize
    def __init__(self):
        
        # initialize that the block has not been detected yet
        self.block_detected = False
        
        # initialize the difference limit between the front Top & Bot sensors
        self.front_sensor_diff_limit = 4
        
    def scan_L_for_block(self, sensor_dict):
        ''' 
        input: 
        output: scan the loading zone for the block
        
        Slowly rotate and move around to scan for the block in the loading zone
        '''   
        front_sensor_diff = sensor_dict[0] - sensor_dict[6]
        
        # rotate left until it detects a block
        while front_sensor_diff < self.front_sensor_diff_limit:
            self.move('r0--5')
                
        self.block_detected = True
    
    def scan_R_for_block(self, sensor_dict):
        '''
        input: 
        output:
        '''
        front_sensor_diff = sensor_dict[0] - sensor_dict[6]
        
        # rotate right until it detects a block
        while front_sensor_diff < self.front_sensor_diff_limit:
            self.move('r0-5')
            
        self.block_detected = True  
        
    def check_clearance_to_block(self, sensor_dict):
        '''
        input: sensor dictionary from OA
        output: move forward/backward tiny bit to get better celarance
        '''
        front_sensor_diff = sensor_dict[0] - sensor_dict[6]
        
        if front_sensor_diff > self.front_sensor_diff_limit:
            # move the robot forward/backward to get in proper spot to pick up the block
            if sensor_dict[6] > 5: 
                self.move('w0-0.5')
            elif sensor_dict[6] < 2:
                self.move('w0--0.5')
        
    def pick_up_block(self):
        ''' 
        input: sensor dictionary from OA
        output: pick up the block
        
        A fine-tuned set of movements to move the arm, pick up the block, and hold the block up.
        '''
        # move arm down, open gripper, move forward, close gripper, move arm up
        self.move('a180')
        self.move('go')
        self.move('w0-1')
        self.move('gc')
        self.move('a-180')
        
    def move(self, command):
        '''
        input: command
        output: None
        '''
        PA.write(command)
        
        # may change sleep time
        time.sleep(3)
        
        
            
        
        
        
        
        
        
        
