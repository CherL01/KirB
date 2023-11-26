import math
import time

class BlockDetection():

    # initialize
    def __init__(self):
        
        # initialize that the block has not been detected yet
        self.block_detected = False
        
        # initialize the difference limit between the front Top & Bot sensors
        ### CAN CHANGE
        self.front_sensor_diff_limit = 4

        # initialize that block is not within pick up range
        self.pick_up_range = False
        
        self.prev_reading = None
        self.scan_direction = None

        self.scan_angle = 3

        # initialize front/bot sensor diff due to placement of sensors
        self.front_bot_diff = 0.5
    
    def get_front_bot_diff(self, sensor_dict):
        '''
        input: dictionary with sensor values
        output: difference between front/bot sensors
        '''

        return sensor_dict['u0'] - (sensor_dict['u6'] + self.front_bot_diff)
    
    def scan_for_block(self, sensor_dict, direction):
        ''' 
        input: sensor dictionary, current square label
        output: True if block detected (False otherwise), list of movement commands
        
        Slowly rotate (L/R) to scan for the block in the loading zone
        '''   
        front_bot_sensor_diff = self.get_front_bot_diff(sensor_dict)
        
        # if front/bot sensor diff
        if front_bot_sensor_diff > self.front_sensor_diff_limit:
            self.block_detected = True
            return self.block_detected, ['']
        
        # rotate until it detects a block
        else:
            if direction == 'L':
                self.scan_direction = 'L'
                return self.block_detected, [f'r0--{self.scan_angle}']
            else:
                self.scan_direction = 'R'
                return self.block_detected, [f'r0-{self.scan_angle}']
        
    def check_clearance_to_block(self, sensor_dict):
        '''
        input: sensor dictionary from OA
        output: True if block at a pick up distance (False otherwise), list of movement command (forward/backward) to get better celarance
        '''
        front_bot_sensor_diff = self.get_front_bot_diff(sensor_dict)
        
        if front_bot_sensor_diff > self.front_sensor_diff_limit:
            # move the robot forward/backward to get in proper spot to pick up the block
            
            
            if self.prev_reading is not None and abs(self.prev_reading - sensor_dict['u6']) > 7:
                print("can't find block anymore")
                if self.scan_direction == 'L':
                    self.prev_reading = None
                    return self.pick_up_range, ['r0-3']
                else: 
                    self.prev_reading = None
                    return self.pick_up_range, ['r0--3']
                
            self.prev_reading = sensor_dict['u6']
            
            if sensor_dict['u6'] > 5: 
                return self.pick_up_range, ['w0-0.5']
            
            elif sensor_dict['u6'] < 4:
                return self.pick_up_range, ['w0--0.5']
            
            else:
                self.pick_up_range = True
                return self.pick_up_range, ['']
            
        else:
            self.pick_up_range = True
            return self.pick_up_range, ['']
        
    def pick_up_block(self):
        ''' 
        input: self
        output: list of commands to pick up block
        
        A fine-tuned set of movements to move the arm, pick up the block, and hold the block up.
        '''
        
        # move arm down, move forward 0.5 inches, close gripper, move arm up
        return ['a40', 'w0-0.5', 'gc', 'a180']
    
    def drop_off_block(self):
        ''' 
        input: self
        output: list of commands to drop off block
        
        A fine-tuned set of movements to move the arm, drop off the block, and move arm back up.
        '''
        
        # move back 5 inches to give room, move arm down, open gripper, move back, move arm up
        return ['w0--5', 'a40', 'go', 'w0--1', 'a180']