import time
import math

from KIRB_python_arduino import PyArduino 
from KIRB_localization import MazeLocalization
from KIRB_block_detection import BlockDetection

PA = PyArduino(com_port="COM7")
ML = MazeLocalization()
BD = BlockDetection()

class ObstacleAvoidance():
    '''
    - obstacle avoidance
    - responsible for sending commands and receiving sensor readings
    '''

    def __init__(self):

        self.running = True

        # store sensor labels, sensors, and sensor names in lists/dicts
        # ['FRONT', 'FRONT-LEFT', 'BACK-LEFT', 'FRONT-RIGHT', 'BACK-RIGHT', 'BACK', 'FRONT_BOTTOM']
        self.sensor_label_list = ['u0', 'u1', 'u2', 'u3', 'u4', 'u5', 'u6']
        self.sensor_label2reading_dict = {}
        for l, s in zip(self.sensor_label_list, [0 for _ in range(len(self.sensor_label_list))]):
            self.sensor_label2reading_dict[l] = s

        # initialize movement command dict
        self.left_turn_angle = 95
        self.right_turn_angle = 90
        self.travel_distance = 11.75
        self.mov_dict = {'LT': [f'r0--{self.left_turn_angle}'],
                        'RT': [f'r0-{self.right_turn_angle}'],
                        'F': [f'w0-{self.travel_distance}'],
                        'B': [f'w0--{self.travel_distance}'],
                        'L': [f'r0--{self.left_turn_angle}', f'w0-{self.travel_distance}'],
                        'R': [f'r0-{self.right_turn_angle}', f'w0-{self.travel_distance}'],}

        #initialize sensor difference and limit and preset values
        self.left_sensor_difference = 0
        self.right_sensor_difference = 0
        self.sensor_difference_limit = 0.15
        self.e_stop_limit = 1.18
        self.square_dim = 12
        self.forward_limit = 2.8
        self.loading_zone_path = None
        self.prev_square = None

    def detect_wall(self, reading):
        '''
        input: sensor reading
        output: True if there is an immediate wall, False otherwise
        '''

        if reading > ML.wall_limit:
            return False
        
        else:
            return True

    def localizable_square_detection(self):
        '''
        input: self
        output: True if navigated to a square that is localizable, False otherwise
        '''

        # get all sensor readings
        sensor_list = self.get_sensor_readings()
        front_sensor = self.sensor_label2reading_dict['u0']
        back_sensor = self.sensor_label2reading_dict['u5']

        # if wall not detected in front, return False
        if not self.detect_wall(front_sensor):
            return False
        
        # if wall detected in front
        else:

            # if back wall detected
            if self.detect_wall(back_sensor):
                print('back wall detected, turning right')
                self.move('r0-90')
                return False
            
            elif ML.loading_zone is True:
                if sensor_list[1] < ML.wall_limit or sensor_list[3] < ML.wall_limit:
                    print('in an L shaped square, ready to localize!')
                    return True
                else:
                    self.move('r0-90')
                    return False
            
            else:
                return True
    
    def avg_sensor_reading(self, side): 
        '''
        input: side of interest (L or R)
        output: average sensor reading
        '''

        if side == 'L':
            s1 = self.sensor_label2reading_dict['u1']
            s2 = self.sensor_label2reading_dict['u2']
            if abs(s1 - s2) > ML.wall_limit:
                return max(s1, s2)

        else:
            s1 = self.sensor_label2reading_dict['u3']
            s2 = self.sensor_label2reading_dict['u4']
            if abs(s1 - s2) > ML.wall_limit:
                return max(s1, s2)

        average = (s1 + s2) / 2
            
        return average
    
    def get_sensor_readings(self):
        '''
        input: self
        output: list of sensor readings in order of F, L, B, R 
            - can be used as input to localization

        note: also stores sensor readings in self.sensor_label2reading_dict 
        '''

        # split sensor readings and store in init variables
        # time.sleep(5)
        for _ in range(3):
            
            # get message from buffer
            message = PA.blocking_read()

            # print('message: ', message)
            
            try:
                # print('message split: ', message.split('|')[1:8])
                for sensor in message.split('|')[1:8]:
                    label, reading = sensor.split('=')
                    # print("label/reading: ", label, reading)
                    self.sensor_label2reading_dict[f'u{int(label)}'] = float(reading)

                if self.sensor_label2reading_dict['u0'] == 0:
                    self.get_sensor_readings()

            except ValueError:
                self.get_sensor_readings()

        print('\nsensor reading dict: ', self.sensor_label2reading_dict)

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
        if 'r' in command and (str(self.left_turn_angle) in command or str(self.right_turn_angle) in command):
            # self.move('x')
            self.check_turn_clearance()
            # self.move('s')

        PA.write(command)
        time.sleep(3)

    def check_turn_clearance(self):
        '''
        input: self
        output: True if there is clearance
        '''

        cleared = False
        front_turn_limit = 2.0
        sides_turn_limit = 2.76
        
        sensor_list = self.get_sensor_readings()    # 4 values for each side of the rover [F,L,B,R]
        
        # no wall in front
        if sensor_list[0] > (ML.wall_limit + ML.sensor_tolerance):
            # if one tile ahead
            if sensor_list[0] < 24:
                front_turn_limit = 14.5
            # if two tiles ahead
            elif sensor_list[0] < 36:
                front_turn_limit = 26.5
            # if three tiles ahead
            elif sensor_list[0] < 48:
                front_turn_limit = 38.5
        
        while True:

            if sensor_list[0] < front_turn_limit or sensor_list[2] < self.e_stop_limit:

                print('\nnot enough clearance')
                
                print('front back turn limit: ', front_turn_limit)
                print('sensor list: ', sensor_list)
                
                # if the front is too close, move back. if the back is too close, move up. if too far from front, move up a bit
                if sensor_list[0] < front_turn_limit:
                    self.move("w0--1")
                    # self.parallel()

                elif front_turn_limit == 2.0 and sensor_list[0] > 3.54:
                    self.move("w0-0.5")

                elif sensor_list[2] < self.e_stop_limit:
                    self.move("w0-0.75")
                    # self.parallel()
                    
                sensor_list = self.get_sensor_readings()
                        
                # no wall in front
                if sensor_list[0] > (ML.wall_limit + ML.sensor_tolerance):
                    # if one tile ahead
                    if sensor_list[0] < 24:
                        front_turn_limit = 14.5
                    # if two tiles ahead
                    elif sensor_list[0] < 36:
                        front_turn_limit = 26.5
                    # if three tiles ahead
                    elif sensor_list[0] < 48:
                        front_turn_limit = 38.5
                else:
                    front_turn_limit = 2.0

            if sensor_list[1] < sides_turn_limit or sensor_list[3] < sides_turn_limit:
                print('not enough side clearance')
                # if the sides are too close, adjust                
                if sensor_list[1] < sides_turn_limit:
                    self.move('x')  # turn off parallel
                    self.move('r0--20')
                    self.move('w0--1.5')
                    self.move('r0-20')
                    self.move('w0-1.25')
                    # self.move('s')  # turn on parallel
                    # # self.parallel()
                elif sensor_list[3] < sides_turn_limit:
                    self.move('x')  # turn off parallel
                    self.move('r0-20')
                    self.move('w0--1.5')
                    self.move('r0--20')
                    self.move('w0-1.25')
                    # self.move('s')  # turn on parallel
                    # # self.parallel()
                
                sensor_list = self.get_sensor_readings()
                        
                # no wall in front
                if sensor_list[0] > (ML.wall_limit + ML.sensor_tolerance):
                    # if one tile ahead
                    if sensor_list[0] < 24:
                        front_turn_limit = 14.5
                    # if two tiles ahead
                    elif sensor_list[0] < 36:
                        front_turn_limit = 26.5
                    # if three tiles ahead
                    elif sensor_list[0] < 48:
                        front_turn_limit = 38.5
                else:
                    front_turn_limit = 2.0

            else:
                break
                    
        cleared = True     
        print('cleared: ', cleared)

        # start parallel
        self.move('s')
        
        return cleared

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

    # def parallel(self, initial=False):
    #     '''
    #     input: self
    #     return: False if emergency stop activated, True otherwise
    #     '''
    #     print('\nrunning parallel!')
    #     # print('sensor reading dict (in parallel): ', self.sensor_label2reading_dict)

    #     _ = self.get_sensor_readings()
    #     self.sensor_diff()
        
    #     closest_sensor, _ = self.get_closest()
    #     print('closest sensor: ', closest_sensor)

    #     print('left sensor diff: ', self.left_sensor_difference)
    #     print('right sensor diff: ', self.right_sensor_difference)
        
    #     #########
    #     #check left & right side difference to centre itself
        
    #     l_sense = (self.sensor_label2reading_dict['u1']+self.sensor_label2reading_dict['u2'])/2
    #     r_sense = (self.sensor_label2reading_dict['u3']+self.sensor_label2reading_dict['u4'])/2
    #     #case when KIRB is in hallway
    #     if (l_sense + r_sense) < 10:
    #         print('in a hallway!')
            
    #         #when left is more open
    #         if l_sense > r_sense + 2:
    #             print('moving to left of hallway')
    #             self.move('r0--6')
            
    #         #when right is more open
    #         if r_sense > l_sense + 2:
    #             print('moving to right of hallway')
    #             self.move('r0-6')
                
    #     #############
                
    #     # both sides are over sensor difference limit (not parallel)
    #     if not (self.left_sensor_difference < self.sensor_difference_limit and self.right_sensor_difference < self.sensor_difference_limit):
            
    #         print("Both sides over sensor limit")

    #         # CONDITION #1: 45deg placement
    #         # TURN 4 DEG WHEN NOT ALIGNED
    #         # front left is closest
    #         if closest_sensor == 'u1':
    #             print('front left closest: moving right')
    #             self.move('r0-6')
        
    #         # back left is closest
    #         elif closest_sensor == 'u2':
    #             print('back left closest: moving left')
    #             self.move('r0--6')

    #         # front right is closest
    #         elif closest_sensor == 'u3':
    #             print('front right closest: moving left')
    #             self.move('r0--6')

    #         # back right is closest
    #         elif closest_sensor == 'u4':
    #             print('back right closest: moving right')
    #             self.move('r0-6')
                
    #     if (self.avg_sensor_reading('L') < self.e_stop_limit) or (self.avg_sensor_reading('R') < self.e_stop_limit):
    #         print('OOPS too close to a wall :(')
    #         self.check_turn_clearance()

    #     if initial is True:
    #         print('WHERE AM I? I HAVE TO FIND MYSELF! "Just keep swimming :)"')
    #         # if there is room in front, travel forward one inch
    #         if self.sensor_label2reading_dict['u0'] >= self.forward_limit:
    #             self.move('w0-4')  
    #         else:
    #             self.move('w0--2')

    def initial_navigation(self):
        '''
        input: self
        output: None
         
        runs until initial navigation is completed
        '''

        # move arm up
        print('moving arm up...')
        self.move('a180')

        # travels through the maze purely on obstale avoidance until robot is in a localizable square
        print('\nstart parallel')
        self.move('start')
        while True:
            # print('in loop')
            if self.localizable_square_detection() is True:
                break
            # print('\nattempt to run parallel')
            print('\nattempt to go straight')
            self.move('w0-2.75')
            # self.parallel(initial=True)

        print('localizable square found!')
        
    def localize_and_navigate(self, zone, location=None):
        '''
        input: zone for navigation
        output: None

        localizes robot and navigates to loading zone or drop off zone
        '''

        # get sensor readings in a list
        sensors_list = self.get_sensor_readings()
        print('\nsensor list (localize and navigate): ', sensors_list)

        # run localization in initial square 
        while ML.initial is True:

            # # check wall locations, turn off parallel if at 3 or 4 way intersection
            # wall_locs = [1 if sensor_value <= ML.wall_limit else 0 for sensor_value in sensors_list]
            # if sum(wall_locs) <= 1:
            #     print('3/4-way intersection, stop parallel')
            #     self.move('x')
            #     self.parallel()

            # if in D1, W:
            if (sensors_list[1] < ML.wall_limit) and (sensors_list[2] > 42): # and (sensors_list[3] > 30)
                print('in D1 (W), changed back sensor value')
                sensors_list[2] = 60
                print('changed sensor list: ', sensors_list)

            # if in D1, S or A1, w:
            elif (sensors_list[3] < ML.wall_limit) and (sensors_list[2] > 35) and (sensors_list[2] < 42) and (sensors_list[1] > 30): # and (sensors_list[2] < 42):
                if ML.loading_zone is False:
                    print('in D1 (S), changed left sensor value')
                    sensors_list[1] = 60
                    print('changed sensor list: ', sensors_list)

                elif ML.loading_zone is True:
                    print('could be in A1 (W), did not change sensor values')

            # if in D6, S OR A1, N:
            elif (sensors_list[1] < ML.wall_limit) and (sensors_list[3] > 37): # (36 < sensors_list[2] < 42) and 
                if ML.loading_zone is False:
                    print('in D6 (S), changed right sensor')
                    sensors_list[3] = 60
                    print('changed sensor list: ', sensors_list)

                elif ML.loading_zone is True:
                    print('could be in A1 (N), did not change sensor values')

            # if in D6, E:
            elif (sensors_list[3] < ML.wall_limit) and (sensors_list[1] > 26) and (sensors_list[2] > 30):
                print('in D6 (E), changed back sensor')
                sensors_list[2] = 60
                print('changed sensor list: ', sensors_list)

            _, _, command_loc = ML.initial_localize(sensors_list)
                
            if command_loc == ['']:
                print('initial localization done!')

                # # start parallel
                # print('restarting parallel')
                # self.move('s')
                break

            command_ard = self.convert_command(command_loc[0])
                
            for command in command_ard:
                print('initial localize command: ', command)
                self.move(command)
                # self.parallel()
                
            # get sensor readings in a list
            sensors_list = self.get_sensor_readings()
            print('sensor list (initial localize): ', sensors_list)
                
            if command_loc != ['RT']:
                print('initial localization done!')

                # # start parallel
                # print('restarting parallel')
                # self.move('s')

                break
        
        # loops until localization is fully complete
        while ML.localized is False:

            # sensors_list = self.get_sensor_readings()

            # # check wall locations, turn off parallel if at 3 or 4 way intersection
            # wall_locs = [1 if sensor_value <= ML.wall_limit else 0 for sensor_value in sensors_list]
            # if sum(wall_locs) <= 1:
            #     print('3/4-way intersection, stop parallel')
            #     self.move('x')
            #     self.parallel()

            localized, _, command_loc = ML.localize(sensors_list)
            print('localized: ', localized)

            if localized:
                break

            command_ard = self.convert_command(command_loc[0])

            for command in command_ard:
                self.move(command)
                # self.parallel()

            # # start parallel
            # print('start parallel')
            # self.move('s')

        # navigate to loading zone
        if zone == 'loading zone':
            path, movements = ML.lz_navigation()
            self.loading_zone_path = path
            print('loading zone path: ', path)

            # get square, heading, and navigation command
            for square, (command_nav, heading) in zip(path[1:], movements):
                print('next square and heading: ', square, heading)

                # # start parallel
                # print('start parallel')
                # self.move('s')

                # turn off parallel at 4 way intersection
                if self.prev_square == 'B6':
                    print('at 4 way, turning off parallel')
                    self.move('x')

                command_ard = self.convert_command(command_nav[0])

                for command in command_ard:

                    print('command (during navigation): ', command)
                    self.move(command)

                # turn on parallel after 4 way intersection
                if self.prev_square == 'B6':
                    print('leaving 4 way, turning on parallel')
                    self.move('start')

                sensors_list = self.get_sensor_readings()

                self.prev_square = square

                # # check wall locations, turn off parallel if at 3 or 4 way intersection
                # wall_locs = [1 if sensor_value <= ML.wall_limit else 0 for sensor_value in sensors_list]
                # if sum(wall_locs) <= 1:
                #     print('3/4-way intersection, stop parallel')
                #     self.move('x')
                #     self.parallel()

            print('reached loading zone!')
            self.move('led1')
            print('loading zone - current location: ', ML.current_location)

        # navigate to drop off zone
        elif zone == 'drop off zone':

            path, movements = ML.doz_navigation(location)
            print('drop off zone path: ', path)

            # get square, heading, and navigation command
            for square, (command_nav, heading) in zip(path[1:], movements):
                print('next square and heading: ', square, heading)

                # # start parallel
                # print('start parallel')
                # self.move('s')

                # turn off parallel at 4 way intersection
                if self.prev_square == 'B6':
                    print('at 4 way, turning off parallel')
                    self.move('x')

                command_ard = self.convert_command(command_nav[0])

                for command in command_ard:

                    print('command (during navigation): ', command)
                    self.move(command)

                # turn on parallel after 4 way intersection
                if self.prev_square == 'B6':
                    print('leaving 4 way, turning on parallel')
                    self.move('start')

                sensors_list = self.get_sensor_readings()

                self.prev_square = square

                # # check wall locations, turn off parallel if at 3 or 4 way intersection
                # wall_locs = [1 if sensor_value <= ML.wall_limit else 0 for sensor_value in sensors_list]
                # if sum(wall_locs) <= 1:
                #     print('3/4-way intersection, stop parallel')
                #     self.move('x')
                #     self.parallel()

            print('reached drop off zone!')
            self.move('led3')
            
            print('disabled parallel')
            self.move('x')

    def block_detect_and_move(self):
        '''
        input: self
        output: None

        runs until block is detected and picked up
        '''

        # stop parallel
        print('\ndisabled parallel')
        self.move('x')
        
        self.check_turn_clearance()
        
        # stop parallel
        print('\ndisabled parallel')
        self.move('x')

        # scan for block
        print('scanning for block')
        turns = 0
        direction_changed = False
        BD.block_detected = False
        current_square = self.loading_zone_path[-1]
        print('current square: ', current_square)
        while True:

            # get sensor readings and current square in loading zone
            sensor_list = self.get_sensor_readings()
            print('sensor list: ', sensor_list)

            # if current square is B1, start by scanning right
            if current_square == 'B1':

                # if sensor_list[3] < 35:
                if direction_changed is False: # sensor_list[0] < 25:
                    direction = 'R'
                    turns += 1

                # if right sensor reads more than 20, scan left
                else:
                    direction = 'L'
                    turns += 1
                    
            # if current square is A2, start by scanning left
            elif current_square == 'A2':

                # if sensor_list[1] < 35:
                if direction_changed is False: # sensor_list[0] < 25:
                    direction = 'L'
                    turns += 1

                # if left sensor reads more than 20, scan right
                else:
                    direction = 'R'
                    turns += 1

            # run block detection
            print('direction: ', direction)
            detected, commands = BD.scan_for_block(self.sensor_label2reading_dict, direction)

            # if block detected, move to next stage
            if detected is True:
                print('block detected!')
                break

            # block not detected, carry out detection commands
            for command in commands:

                print('command (during block detection): ', command)
                self.move(command)
                
            print('number of turns (during scanning): ', turns)

            if turns > 15:
                direction_changed = True
                
                print('cant find block, gonna move')
                
                if current_square == 'A2':
                    self.move('r0--10')
                    self.move('s')
                    self.move('w0-11')
                    self.move('r0-75')
                    self.move('x')
                    turns = 0
                    
                else:
                    self.move('r0-10')
                    self.move('s')
                    self.move('w0-11')
                    self.move('r0--75')
                    self.move('x')
                    turns = 0
                    
                
            # if sensor_list[0] > 25:
            #     if current_square == 'B1':
            #         angle = 90 - BD.scan_angle * turns
            #         self.move(f'r0-{angle}')
            #         turns = 0
            #         direction_changed = True
                    
            #     else:
            #         angle = 90 - BD.scan_angle * turns
            #         self.move(f'r0--{angle}')
            #         turns = 0
            #         direction_changed = True
                    
        # ### TESTING (DID NOT WORK:(( )
        # movements = BD.calculate_movements(self.sensor_label2reading_dict, direction)
        # for command in movements:

        #     print('command (new movement function): ', command)
        #     self.move(command)

        # check if block is centered
        print('\ncentering')
        centering_turns = 0
        while True:
            # get sensor readings
            sensor_list = self.get_sensor_readings()

            # initial direction should be from prev loop
            print('direction of turn: ', direction)
            print('previous reading: ', BD.prev_reading)
            print('previous previous reading: ', BD.prev_prev_reading)
            centered, commands = BD.check_centered(self.sensor_label2reading_dict, direction)

            if centered is True:
                print('block is centered!')
                break

            # block not centered, carry out movement commands
            for command in commands:

                print('command (centering): ', command)
                self.move(command)

            # if centered is True:
            #     print('block is centered!')
            #     break

            centering_turns += 1

            # if turned 3 times already, change directions
            if centering_turns == 3:

                if direction == 'L':
                    centering_turns = 0
                    direction = 'R'
                    print('changed turn direction to: ', direction)

                else:
                    centering_turns = 0
                    direction = 'L'
                    print('changed turn direction to: ', direction)

        # block detected, will move so block is in pick up range
        print('\nmoving to pick up block')

        while True:
            # get sensor readings
            sensor_list = self.get_sensor_readings()

            # run check clearance
            in_range, commands = BD.check_clearance_to_block(self.sensor_label2reading_dict)
            print('bot sensor: ', self.sensor_label2reading_dict['u6'])
            print('in range: ', in_range)

            # block is in range, move to next stage
            if in_range is True:
                print('block is in range!')
                break

            # block not in range, carry out movement commands
            for command in commands:

                print('command (moving to pick up range): ', command)
                self.move(command)

            if abs(self.sensor_label2reading_dict['u0'] - self.sensor_label2reading_dict['u6']) < 0.2:
                self.block_detect_and_move()
                
        # # check if block is centered
        # print('\nrecentering after moving')
        # centering_turns = 0
        # while True:
        #     # get sensor readings
        #     sensor_list = self.get_sensor_readings()

        #     # initial direction should be from prev loop
        #     print('direction of turn: ', direction)
        #     print('previous reading: ', BD.prev_reading)
        #     print('previous previous reading: ', BD.prev_prev_reading)
        #     centered, commands = BD.check_centered(self.sensor_label2reading_dict, direction)

        #     if centered is True:
        #         print('block is centered!')
        #         break

        #     # block not centered, carry out movement commands
        #     for command in commands:

        #         print('command (centering): ', command)
        #         self.move(command)

        #     # if centered is True:
        #     #     print('block is centered!')
        #     #     break

        #     centering_turns += 1

        #     # if turned 4 times already, change directions
        #     if centering_turns == 3:

        #         if direction == 'L':
        #             centering_turns = 0
        #             direction = 'R'
        #             print('changed turn direction to: ', direction)

        #         else:
        #             centering_turns = 0
        #             direction = 'L'
        #             print('changed turn direction to: ', direction)

        # block is in range, pick up block
        print('picking up block...')
        commands = BD.pick_up_block(current_square)

        for command in commands:

            print('command (pick up): ', command)
            self.move(command)
        
        # block picked up
        self.move('led2')

    def block_drop_off(self):
        '''
        input: self
        output: None

        Drops block off in drop off location
        '''

        # drop off zone reached, drop off block
        print('\ndropping off block...')
        commands = BD.drop_off_block()
        for command in commands:

            print('command (drop off): ', command)
            self.move(command)

        print('dropped off block!')
        print('complete')


drop_off_loc = 'A6'
testing = False

OA = ObstacleAvoidance()

# # # navigates to a localizable square
OA.initial_navigation()

# # # tries to localize then travel to loading zone
OA.localize_and_navigate('loading zone')

# start block detection
if testing is True:
    OA.loading_zone_path = ['B1']
    ML.loading_zone = True
OA.block_detect_and_move()

# renavigate to a localizable square
OA.initial_navigation()

# # tries to localize then travel to drop off zone
OA.localize_and_navigate('drop off zone', drop_off_loc)

# # # drop off block
OA.block_drop_off()
