import math
import numpy as np
import collections

class MazeLocalization():

    # initialize maze coordinates
    def __init__(self):

        # initialize that robot is not localized
        self.localized = False

        # initialize that robot is on the initial square of localization
        self.initial = True
        
        # maze labels
        self.maze_labels = [[f'{y}{x}' for x in range(1, 9)] for y in ['A', 'B', 'C', 'D']]
        
        # maze wall configurations
        self.wall_configs = [
            [2, 1, 3, 2, math.inf, 4, math.inf, 4],
            [1, 2, math.inf, 2, 3, 0, 3, 1],
            [3, math.inf, 4, math.inf, math.inf, 3, math.inf, 3],
            [2, 3, 1, 3, 3, 2, math.inf, 4]
        ]

        # dict mapping maze labels with wall configurations
        self.maze_map = {}
        for l, c in zip(self.maze_labels, self.wall_configs):
            for label, value in zip(l, c):
                self.maze_map[label] = value 

        # maze labels <-> coordinates
        mazeLabels_flat = [f'{y}{x}' for x in range(1, 9) for y in ['A', 'B', 'C', 'D']]
        self.labels2coords_y_x = {label: coord for label, coord in zip(mazeLabels_flat, [(y, x) for x in range(8) for y in range(4)])}
        self.coords_y_x2labels = {coord: label for label, coord in self.labels2coords_y_x.items()}
        
        # initialize current location, coordinates, potential square heading pairs, last movement command given
        self.current_location = None
        self.current_location_coords = None
        self.square_heading_pairs = None
        self.last_movement = None

        # initialize whether loading zone and drop off zones have been reached
        self.loading_zone = False
        self.drop_off_zone = False
        
        # square dimension (inches)
        self.square_dim = 12

        # tolerance for sensor readings (CAN CHANGE)
        self.sensor_tolerance = 2

        # limit for wall detection
        self.wall_limit = 4.5

        # sensor noise
        self.sensor_noise = 5

        # probability threshold (diff between highest and second highest prob for robot to be considered localized)
        self.prob_threshold = 0.6
    
    def make_movement(self, current_loc, heading, mov_direction, turn = True):
        """ 
        input: current location, heading, movement direction, (turn) if heading should change 
        output: new location, new heading; None if out of bounds or a wall

        Returns new_location and new_heading of the car given the current location,
        the direction the car is facing (N W S E), and the movement direction (F B L R)
            
        Assumes that the car turns for left and right movements, and keeps the same heading 
        for front and back movements

        Returns None for new_location and new_direction if the location is out of bounds or in a wall
        """
        
        # movements if heading is north
        if heading == "N":
            y_x_movement = {"F": [-1, 0],
                            "B": [1, 0],
                            "L": [0, -1],
                            "R": [0, 1],}

        # movements if heading is west
        elif heading == "W":
            y_x_movement = {"F": [0, -1],
                            "B": [0, 1],
                            "L": [1, 0],
                            "R": [-1, 0],}
        
        # movements if heading is south
        elif heading == "S":
            y_x_movement = {"F": [1, 0],
                            "B": [-1, 0],
                            "L": [0, 1],
                            "R": [0, -1],}
        
        # movements if heading is east
        elif heading == "E":
            y_x_movement = {"F": [0, 1],
                            "B": [0, -1],
                            "L": [-1, 0],
                            "R": [1, 0],}
        
        else:
            raise ValueError()
    
        # get y, x movements and move accordingly
        movement_Y, movement_X = y_x_movement[mov_direction]
        current_Y, current_X = self.labels2coords_y_x[current_loc]

        new_y = movement_Y + current_Y
        new_x = movement_X + current_X

        # out of bounds, return None
        if (new_x >= 8) or (new_x < 0) or (new_y >= 4) or (new_y < 0):
            return None, None
        
        new_loc = self.coords_y_x2labels[(new_y, new_x)]

        # wall, return None
        if math.isinf(self.maze_map[new_loc]):
            return None, None
        
        new_heading = heading

        # determine new heading if turn is true
        if turn is True:
            headings = ["N", "W", "S", "E"]
            idx = headings.index(heading)
            if mov_direction == "L":
                idx += 1
            elif mov_direction == "R":
                idx -= 1
            elif mov_direction == "F":
                pass
            elif mov_direction == "B":
                pass
            else:
                raise ValueError()
            idx = idx % 4
            new_heading = headings[idx]

        return new_loc, new_heading
    
    def theoretical_sensor_readings(self, square_label, heading):
        '''
        input: square label, heading (direction: N, S, E, W)
        output: theoretical sensor readings (F, L, B, R)
        '''
        
        # set up potential movement directions and theoretical values list
        movement_directon = ['F', 'L', 'R', 'B']
        theoretical_values = []

        # recursively find theoretical sensor readings for each movement direction
        for d in movement_directon:

            # start with current square and a direction, travel one square until we hit a wall
            # calculate how many loops (squares) before hitting a wall
            current_loc = square_label
            num_loops = 0
            while True:
                new_loc, _ = self.make_movement(current_loc, heading, d, turn=False)

                if new_loc == None:
                    break

                current_loc = new_loc
                num_loops += 1
            
            # multiply number of squares by distance to get theoretical reading
            # added sensor tolerance for extra distance since measurement from sensor isnt directly on the edge of a square
            theoretical_values.append(num_loops * self.square_dim + self.sensor_tolerance)

        return theoretical_values
    
    def potential_square_heading_pairs(self, sensor_readings):
        '''
        input: list of sensor readings (F, L, B, R)
        output: list of tuples of potential squares labels and their respective headings (N, S, E, W), (ROTATION SHOULD ALWAYS BE FALSE IN NEW MODIFICATION) if rotation is required

        note: this function is for initial conditions ONLY (after robot travels to a wall to ensure that it is in a square)
        '''

        # variable to determine if robot requires 
        rotate = False

        # check wall locations
        wall_locs = [1 if sensor_value <= self.wall_limit else 0 for sensor_value in sensor_readings]

        # case 1: one wall at F (wall config = 1)
        # potential squares: B8, D3 ; A2, B1 (loading zone)
        if sum(wall_locs) == 1:
            potential_squares = ['A2', 'B1', 'B8', 'D3']
            headings = ['N', 'W', 'E', 'S']

        # case 2: two walls at F + L (wall config = 2)
        # IF WALLS AT F + R, ROTATE RIGHT SO WALLS ARE AT F + L
        # potential squares: A4, B4, D1, D6 ; A1, B2 (loading zone)
        elif sum(wall_locs) == 2:

            # if walls at F + R, prompt robot to rotate 90 deg to the right
            if wall_locs[1] == 0:
                # rotate = True 
                potential_squares = ['A1', 'B2', 'A4', 'B4', 'D1', 'D6']
                headings = ['W', 'E', 'N', 'S', 'S', 'E']

            else:

                potential_squares = ['A1', 'B2', 'A4', 'B4', 'D1', 'D6']
                headings = ['N', 'S', 'E', 'W', 'W', 'S']

        # case 3: three walls at F + L + R (wall config = 4)
        # potential squares: A6, A8, C3, D8
        elif sum(wall_locs) == 3:
            potential_squares = ['A6', 'A8', 'C3', 'D8']
            headings = ['N', 'N', 'N', 'S']

        square_heading_pairs = list(zip(potential_squares, headings))

        return square_heading_pairs, rotate
            
    def gaussian_prob(self, mu, sigma, x):
        '''
        input: mu, sigma, x
        output: probability of x for 1-dim Gaussian with mean mu and var sigma
        '''

        # return math.exp( -((mu - x) ** 2) / (sigma ** 2) / 2.) / math.sqrt( 2. * math.pi * (sigma ** 2)) 
        denom = (2*math.pi*sigma)**.5
        num = math.exp(-(float(x)-float(mu))**2/(2*sigma))
        return num/denom

    def square_prob(self, sensor_readings, square_heading_pairs):
        '''
        input: list of sensor readings (F, L, B, R), list of tuples of potential squares labels and their respective headings
        output: list of reordered square heading pairs (in descending probability), list of corresponding probabilities
        '''

        probs = []

        # calculate gaussian probability for each potential square
        for square, heading in square_heading_pairs:
            t_values = self.theoretical_sensor_readings(square, heading)
            print('theoretical values - square: ', square, t_values)
            w = 1.
            for t, s in zip(t_values, sensor_readings):
                w *= self.gaussian_prob(t, self.sensor_noise, s)
            probs.append(w + 1.e-300)

        pairs_w_probs = sorted(list(zip(square_heading_pairs, probs)), key=lambda x:x[1], reverse=True)

        square_heading_pairs_reordered, probs_reordered = list(zip(*pairs_w_probs))

        return square_heading_pairs_reordered, list(np.array(probs_reordered) / sum(probs_reordered))
    
    def prob_after_moving(self, movement, sensor_readings_new, square_heading_pairs):
        '''
        input: movement from robot, list of new sensor readings after robot moves, 
            list of tuples of original potential squares labels and their respective headings
        output: list of new square heading pairs (in descending probability), list of corresponding probabilities
        '''

        # get new square heading pairs
        new_square_heading_pairs = []
        for square, heading in square_heading_pairs:
            # print('(before loop) new square/heading, movement: ', square, heading, movement)
            new_square, new_heading = self.make_movement(square, heading, movement, turn=True)
            if new_square is not None:
                # print('new square/heading: ', new_square, new_heading)
                new_square_heading_pairs.append((new_square, new_heading))

        probs = []
        # calculate gaussian probability for each new square
        for square, heading in new_square_heading_pairs:
            if square != None:
                t_values = self.theoretical_sensor_readings(square, heading)
                w = 1.
                for t, s in zip(t_values, sensor_readings_new):
                    w *= self.gaussian_prob(t, self.sensor_noise, s)
                probs.append(w + 1.e-300)
            else: 
                probs.append(1.e-300)

        pairs_w_probs = sorted(zip(new_square_heading_pairs, probs), key=lambda x:x[1], reverse=True)

        new_square_heading_pairs_reordered, probs_reordered = list(zip(*pairs_w_probs))

        return new_square_heading_pairs_reordered, list(np.array(probs_reordered) / sum(probs_reordered))

    def neighbours(self, square):
        '''
        input: square label
        return: list of neighbouring squares
        '''
        # get coordinate of potential location
        y, x = self.labels2coords_y_x[square]

        # find neighbouring non-diagonal squares
        neighbouring_squares = [self.maze_labels[r][c] for r,c in ((y-1,x), (y+1,x), (y,x-1), (y,x+1)) if 0 <= r < (len(self.maze_labels)) and 0 <= c < (len(self.maze_labels[0])) and self.maze_labels[r][c] is not None ]
        
        return neighbouring_squares

    def bfs(self, des_locs):
        '''
        input: list of destination locations
        output: shortest path to destination location
        '''

        # initialize queue (FIFO) for paths to a square from current location
        # initialize list of squares seen
        queue = collections.deque([[self.current_location[0]]])
        seen = set([self.current_location[0]])

        # loop through to check each potential path
        while queue:

            # get first element (path) of queue
            path = queue.popleft()
            
            # get label of newest square
            label = path[-1]

            # check if newest square is the destination square
            if label in des_locs:
                return path
            
            # get neighbours
            neighbouring_squares = self.neighbours(label)

            # if square has not been seen AND is not a wall, add to queue
            for s in neighbouring_squares:
                if s not in seen:
                    seen.add(s)
                    if self.maze_map[s] != math.inf:
                        queue.append(path + [s])
                    
    def get_movement(self, sensor_readings):
        '''
        input: list of sensor readings
        output: movement command
        
        note: this function gives the direction of the next square for the robot to move to for localization
            (order: L, F, R, B)
        '''

        # define order of movements
        movements = ['L', 'F', 'R', 'B']

        # if no wall on left side
        if sensor_readings[1] > self.wall_limit:
            return movements[0]
        
        # if no wall in front
        elif sensor_readings[0] > self.wall_limit:
            return movements[1]
        
        # if no wall on right side
        elif sensor_readings[3] > self.wall_limit:
            return movements[2]
        
        # if no wall in back
        else:
            return movements[3]

    def get_commands(self, square_heading_pair, path):
        '''
        input: tuple of square label and heading of current location, list of square labels representing a path
        output: list of commands to travel through path in the format of command = (MOVEMENT, HEADING)
        '''

        # initialize list of commands and potential movements
        commands = []
        movements = ['F', 'L', 'B', 'R']

        # start with current location, get movement to travel along path
        label, heading = square_heading_pair
        for square in path[1:]:
            for mov in movements:
                new_loc, new_head = self.make_movement(label, heading, mov, turn=True)
                if new_loc == square:
                    commands.append((mov, new_head))
                    break
            label, heading = new_loc, new_head
        
        return commands
    
    def initial_localize(self, sensor_readings):
        '''
        input: sensor readings
        output: True if robot localized (False otherwise), current location(s), list of movements to complete localization process
        '''

        # # if robot is in inital square (3 initial localization cases)
        # if self.initial == True:

        # get square labels, headings, if rotation is required
        square_heading_pairs, rotation = self.potential_square_heading_pairs(sensor_readings)

        # if rotation is required, return rotation command to main script
        if rotation is True:
            print('\nlocalized (KIRB_localization): ', self.localized)
            return self.localized, square_heading_pairs, ['RT'] # RT = right turn, can change right turn command later to fit generalized command

        # get probabilities for all potential square label and heading pairs
        square_heading_pairs_reordered, square_probs = self.square_prob(sensor_readings, square_heading_pairs)
        print('square heading pairs reordered: ', square_heading_pairs_reordered)
        print('square probs: ', square_probs)
        
        self.square_heading_pairs = square_heading_pairs_reordered

        if (square_probs[0] - square_probs[1]) > self.prob_threshold:
            # set localized as true to continue to navigation
            self.localized = True
            self.current_location = self.square_heading_pairs[0]
            print('current location: ', self.current_location)
        
            return self.localized, self.current_location, ['']

        # set initial as False because will move from initial square
        self.initial = False

        # get movement command to continue localization
        movement = self.get_movement(sensor_readings)
        print('next movement: ', movement)
        self.last_movement = movement

        return self.localized, self.square_heading_pairs, movement
    
    def localize(self, sensor_readings):
        '''
        input: sensor readings
        output: True if robot localized (False otherwise), current location(s), list of movements to complete localization process
        '''
        
        print('\n(localize) sensor readings: ', sensor_readings)
        
        # get probabilities for new potential square label and heading pairs
        new_square_heading_pairs_reordered, new_square_probs = self.prob_after_moving(self.last_movement, sensor_readings, self.square_heading_pairs)
        print('new square heading pairs: ', new_square_heading_pairs_reordered)
        print('new square prob: ', new_square_probs)
        self.square_heading_pairs = new_square_heading_pairs_reordered

        # if one probability is significantly higher than the rest of the probabilities, then robot has localized
        if (new_square_probs[0] - new_square_probs[1]) > self.prob_threshold:
            # set localized as true to continue to navigation
            self.localized = True
            self.current_location = self.square_heading_pairs[0]
            print('current location: ', self.current_location)
        
            return self.localized, self.current_location, ['']
        

        # get movement command to continue localization
        movement = self.get_movement(sensor_readings)
        print('continue localization - next movement: ', movement)
        self.last_movement = movement

        return self.localized, self.square_heading_pairs, movement

    def lz_navigation(self):
        '''
        input: self
        output: path to loading zone, list of movements to complete navigation process
        '''    

        # get the shortest path to loading zone
        loading_zone_path = self.bfs(['A2', 'B1'])
        path_commands = self.get_commands(self.current_location, loading_zone_path)

        # once path for loading zone is obtained, set loading zone to True, reset initial and localized variables
        self.loading_zone = True
        self.initial = True
        self.localized = False
        self.current_location = (loading_zone_path[-1], path_commands[-1][-1])

        return loading_zone_path, path_commands

    def doz_navigation(self, drop_off_loc=None):
        '''
        input: self
        output: path to drop off zone, list of movements to complete navigation process
        '''

        # get the shortest path to drop off zone
        drop_off_zone_path = self.bfs([drop_off_loc])
        path_commands = self.get_commands(self.current_location, drop_off_zone_path)

        return drop_off_zone_path, path_commands

##############################################################################################

###### TEST CASES ######

### test each function

# maze_localization = MazeLocalization()

# test_squares = [f'{y}{x}' for x in range(1, 9) for y in ['A', 'B', 'C', 'D']]
# test_headings = ['N', 'E', 'S', 'W']
# test_movs = ['F', 'L', 'B', 'R']
# test_turns = [True, False]

# # make_movement function
# for s in test_squares:
#     for h in test_headings:
#         for m in test_movs:
#             for t in test_turns:
#                 print(f's: {s}, h: {h}, m: {m}, t: {t}')
#                 print('make movement function result: ', maze_localization.make_movement(s, h, m, turn = t), '\n')


# # theoretical_sensor_readings function
# for s in test_squares:
#     for h in test_headings:
#         print(f's: {s}, h: {h}')
#         print(maze_localization.theoretical_sensor_readings(s, h), '\n')

# # potential_square_heading_pairs function, square_prob function
# # [front sensor, left sensor, back sensor, right sensor]

# # potential_square_heading_pairs function
# test_sensor_readings = [('C3 - N', [3, 2.5, 15, 2.7]), ('D6 - E (should return rotation = True)', [3.5, 40, 60, 4])]
# for s, sr in test_sensor_readings:
    # print('s: ', s)
    # print('sr: ', sr)
    # print(maze_localization.potential_square_heading_pairs(sr), '\n')

# # square_prob function
# test_sensor_readings_after_rotation = [('C3 - N', [3, 2.5, 15, 2.7]), ('D6 - S (after rotation)', [3.5, 4, 40, 60])]
# for s, sr in test_sensor_readings_after_rotation:
#     test_square_heading_pairs, _ = maze_localization.potential_square_heading_pairs(sr)
#     print(maze_localization.square_prob(sr, test_square_heading_pairs))

# # prob_after_moving function
# test_sensor_readings_new = [('D3 - N', ['B'], [15, 25, 4, 32]), ('D5 - W', ['R'], [48, 1, 14, 4])]
# count = 0
# for s, sr in test_sensor_readings_after_rotation:
#     test_square_heading_pairs, _ = maze_localization.potential_square_heading_pairs(sr)
#     test_square_heading_pairs_reordered, test_probs = maze_localization.square_prob(sr, test_square_heading_pairs)
#     m = test_sensor_readings_new[count][1][0]
#     sr_new = test_sensor_readings_new[count][2]
#     print('s: ', s)
#     print('m: ', m)
#     print('sr new: ', sr_new)
#     print('test square heading pairs: ', test_square_heading_pairs)
#     print(maze_localization.prob_after_moving(m, sr_new, test_square_heading_pairs))
#     count += 1

# bfs function
# test_current_locations = [('D5', 'W'), ('D8', 'N'), ('B6', 'W'), ('A1', 'N'), ('D1', 'S'), ('B2', 'E')]
# des_locs = [['A2', 'B1'], ['C3'], ['D8']]
# for cl in test_current_locations:
#     for dl in des_locs:
#         print('cl: ', cl)
#         print('dl: ', dl)
#         maze_localization.current_location = cl
#         print(maze_localization.bfs(dl))

# # get_movement function
# for s, sr in test_sensor_readings_after_rotation:
#     print('s: ', s)
#     print('sr: ', sr)
#     print(maze_localization.get_movement(sr))

# # get_commands function
# for cl in test_current_locations:
#     for dl in des_locs:
#         print('cl: ', cl)
#         print('dl: ', dl)
#         maze_localization.current_location = cl
#         path = maze_localization.bfs(dl)
#         print('path: ', path)
#         print(maze_localization.get_commands(maze_localization.current_location, path))

# # initial_localize, localize, lz_navigation, doz_navigation functions

# count = 0
# print('BEFORE ROTATION')
# for s, sr in test_sensor_readings:
#     maze_localization = MazeLocalization()
#     print('s: ', s)
#     print('sr: ', sr)
#     print('initial: ', maze_localization.initial_localize(sr))

# print('AFTER ROTATION')
# for s, sr in test_sensor_readings_after_rotation:
#     maze_localization = MazeLocalization()
#     print('s: ', s)
#     print('sr: ', sr)
#     print('initial: ', maze_localization.initial_localize(sr))
    
#     print('new square: ', test_sensor_readings_new[count][0])
#     m = test_sensor_readings_new[count][1][0]
#     sr_new = test_sensor_readings_new[count][2]
#     print('sr new: ', sr_new)
#     print('localize: ', maze_localization.localize(sr_new))

#     print('current loc: ', maze_localization.current_location)
#     print('loading zone nagivation: ', maze_localization.lz_navigation())

#     maze_localization.current_location = (maze_localization.lz_navigation()[0][-1], maze_localization.lz_navigation()[1][-1][-1])
#     print('new current loc: ', maze_localization.current_location)

#     for dl in ['A6', 'A8', 'C3', 'D8']:
#         print('dl: ', dl)
#         print('drop off zone navigation: ', maze_localization.doz_navigation(dl))

#     count += 1