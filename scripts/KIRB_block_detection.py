import math
import time

class BlockDetection():

    # initialize
    def __init__(self):
        
        # initialize that the block has not been detected yet
        self.block_detected = False
        
        # initialize the difference limit between the front Top & Bot sensors
        ### CAN CHANGE
        self.front_sensor_diff_limit = 2.5

        # initialize that block is not within pick up range
        self.pick_up_range = False
        
        self.prev_reading = None
        self.prev_prev_reading = None
        self.scan_direction = None
        self.centered = False

        self.scan_angle = 4
        self.bot_limit = 12.5
        self.scan_turns = 0

        # self.initial_scan = True

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
        
        # if front/bot sensor diff is greater than limit AND bot sensor is less than bot limit
        # if front_bot_sensor_diff > self.front_sensor_diff_limit:
        if front_bot_sensor_diff > self.front_sensor_diff_limit and sensor_dict['u6'] < self.bot_limit:
            self.block_detected = True
            return self.block_detected, ['']
        
        # rotate until it detects a block
        else:
            if direction == 'L':
                self.scan_direction = 'L'
                self.scan_turns += 1
                return self.block_detected, [f'r0--{self.scan_angle}']
            else:
                self.scan_direction = 'R'
                self.scan_turns += 1
                return self.block_detected, [f'r0-{self.scan_angle}']
        
    def check_centered(self, sensor_dict, direction):
        '''
        input: sensor dictionary from OA, rotate direction
        output: True if block is centered for pick up, list of movement command (L/R)
        '''

        if self.prev_reading is None:
            self.centered = False

        if self.prev_prev_reading is not None and abs(sensor_dict['u6'] - self.prev_reading) < 0.75 and abs(self.prev_prev_reading - self.prev_reading) < 0.75:
            self.centered = True
            self.prev_reading = None
            self.prev_prev_reading = None
            return self.centered, ['']
            
            # # if direction of movement is left, turn right 
            # if direction == 'L':
            #     return self.centered, ['r0-1.5']
            
            # else:
            #     return self.centered, ['r0--1.5']
        
        self.prev_prev_reading = self.prev_reading
        self.prev_reading = sensor_dict['u6']

        if direction == 'L':
            return self.centered, ['r0--1.75']

        else: 
            return self.centered, ['r0-1.75']
    
    def check_clearance_to_block(self, sensor_dict):
        '''
        input: sensor dictionary from OA
        output: True if block at a pick up distance (False otherwise), list of movement command (forward/backward) to get better celarance
        '''
        # front_bot_sensor_diff = self.get_front_bot_diff(sensor_dict)
        
        # if front_bot_sensor_diff > self.front_sensor_diff_limit:
            # move the robot forward/backward to get in proper spot to pick up the block
            
        if self.prev_reading is not None and abs(self.prev_reading - sensor_dict['u6']) > 7:
            print("can't find block anymore")
            if self.scan_direction == 'L':
                self.prev_reading = None
                return self.pick_up_range, ['r0--2']
            else: 
                self.prev_reading = None
                return self.pick_up_range, ['r0-2']
            
        self.prev_reading = sensor_dict['u6']
        
        forward_distance = sensor_dict['u6'] - 4.75
        
        if sensor_dict['u6'] > 4.75: 
            # return self.pick_up_range, [f'w0-{forward_distance}']
            return self.pick_up_range, ['w0-0.6']
        
        elif sensor_dict['u6'] < 4.25:
            return self.pick_up_range, ['w0--0.5']
        
        else:
            self.pick_up_range = True
            self.prev_reading = None
            return self.pick_up_range, ['']
            
        # else:
        #     self.pick_up_range = True
        #     return self.pick_up_range, ['']
        
    # def calculate_movements(self, sensor_dict, direction):
    #     '''
    #     input: sensor dict from OA, direction of movement
    #     output: list of movements
    #     '''
        
    #     movements = []
    #     bot_distance = sensor_dict['u6']
        
    #     if direction == 'L':
    #         movements.append('r0--1')
    #     else:
    #         movements.append('r0-1')
            
    #     travel_distance = math.floor(bot_distance - 5)
    #     movements.append(f'w0-{travel_distance}')
        
    #     return movements
        
    def pick_up_block(self, current_square):
        ''' 
        input: self
        output: list of commands to pick up block
        
        A fine-tuned set of movements to move the arm, pick up the block, and hold the block up.
        '''
        
        # move arm down, move forward 0.5 inches, close gripper, move arm up
        pick_up_commands = ['a32', 'w0-1', 'gc', 'a180', 'gx']

        if current_square == 'B1':
            angle = 4 * self.scan_turns
            pick_up_commands.append(f'r0--{angle}')

        else:
            angle = 4 * self.scan_turns + 15
            pick_up_commands.append(f'r0-{angle}')

        return pick_up_commands
    
    def drop_off_block(self):
        ''' 
        input: self
        output: list of commands to drop off block
        
        A fine-tuned set of movements to move the arm, drop off the block, and move arm back up.
        '''
        
        # move back 5 inches to give room, move arm down, open gripper, move arm up
        return ['w0--5', 'a40', 'go', 'a180']
    
    # blue, grey, brown, red (top to bot)