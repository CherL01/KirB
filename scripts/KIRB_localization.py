import math
import numpy as np
import collections

class mazeLocalization():

    # initialize maze coordinates
    def __init__(self):
        
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
        
        # initialize current location
        self.current_location = None
        self.current_location_coords = None
        
        # square dimension (inches)
        self.square_dim = 12

        # tolerance for sensor readings (CAN CHANGE)
        self.sensor_tolerance = 2

        # limit for wall detection
        self.wall_limit = 4

        # sensor noise
        self.sensor_noise = 2
    
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
        movement_directon = ['F', 'L', 'B', 'R']
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
        output: list of potential squares labels and their respective headings (N, S, E, W)

        note: this function is for initial conditions ONLY (after robot travels to a wall to ensure that it is in a square)
        '''

        # check wall locations
        wall_locs = [1 if sensor_value <= self.wall_limit else 0 for sensor_value in sensor_readings]

        # case 1: one wall at F (wall config = 1)
        # potential squares: B8, D3 ; A2, B1 (loading zone)
        if sum(wall_locs) == 1:
            potential_squares = ['B8', 'D3']
            headings = ['E', 'S']

        # case 2: two walls at F + L (wall config = 2)
        # IF WALLS AT F + R, ROTATE RIGHT SO WALLS ARE AT F + L
        # potential squares: A4, B4, D1, D6 ; A1, B2 (loading zone)
        elif sum(wall_locs) == 2:

            # if walls at F + R
            if wall_locs[1] == True:
                ### CODE FOR ROTATING ROBOT 90 DEG TO THE RIGHT ###
                pass # delete later

            potential_squares = ['A4', 'B4', 'D1', 'D6']
            headings = ['E', 'W', 'W', 'S']

        # case 3: three walls at F + L + R (wall config = 4)
        # potential squares: A6, A8, C3, D8
        elif sum(wall_locs) == 3:
            potential_squares = ['A6', 'A8', 'C3', 'D8']
            headings = ['N', 'N', 'N', 'S']

        return potential_squares, headings
            
    def gaussian_prob(self, mu, sigma, x):
        '''
        input: mu, sigma, x
        output: probability of x for 1-dim Gaussian with mean mu and var sigma
        '''

        return math.exp( -((mu - x) ** 2) / (sigma ** 2) / 2.) / math.sqrt( 2. * math.pi * (sigma ** 2)) 

    def square_prob(self, sensor_readings, square_heading_pairs):
        '''
        input: list of sensor readings (F, L, B, R), list of potential squares labels and their respective headings
        output: list of corresponding probabilities
        '''

        probs = []

        # calculate gaussian probability for each sensor reading
        for square, heading in square_heading_pairs:
            t_values = self.theoretical_sensor_readings(square, heading)
            w = 1.
            for t, s in zip(t_values, sensor_readings):
                w *= self.gaussian_prob(t, self.sensor_noise, s)
            probs.append(w + 1.e-300)

        return probs
    
    def get_location(self, square_heading_pairs, probabilities):
        '''
        input: list of square heading pairs, list of corresponding probabilities
        output: square heading pair with highest probability
        '''

        # get index of the max probability
        max_prob_index = probabilities.index(max(probabilities))
        self.current_location = square_heading_pairs[max_prob_index][0]
        self.current_location_coords = self.labels2coords_y_x[self.current_location]

        return square_heading_pairs[max_prob_index]

    def neighbours(self, square):
        '''
        input: square label
        return: list of neighbouring squares
        '''
        # get coordinate of potential location
        y, x = self.labels2coords_y_x[square]

        # find neighbouring squares
        neighbouring_squares = [self.maze_labels[r][c] for r in range(y-1 if y > 0 else y, y + 2 if y < len(self.maze_labels)-1 else y + 1) for c in range(x-1 if x > 0 else x, x + 2 if x < len(self.maze_labels[0])-1 else x + 1)]

        # find index of potential location in neighbouring squares
        pot_loc_index = neighbouring_squares.index(square)
        
        # remove potential location
        neighbouring_squares.pop(pot_loc_index)
        
        return neighbouring_squares

    def bfs(self, des_locs):
        '''
        input: list of destination locations
        output: shortest path to destination location
        '''

        # initialize queue (FIFO) for paths to a square from current location
        # initialize list of squares seen
        queue = collections.deque([[self.current_location]])
        seen = set([self.current_location])

        # loop through to check each potential path
        while queue:

            # get first element (path) of queue
            path = queue.popleft()
            
            label = path[-1]

            if label in des_locs:
                return path
            
            neighbouring_squares = self.neighbours(label)

            for s in neighbouring_squares:
                if s not in seen:
                    queue.append(path + [s])
                    seen.add(s)

    # # get potential locations of current square
    # def get_location(self, orientations, new_current_squares=None):
    #     '''
    #     input: 
    #         - list of wall readings at different orientations 
    #             -- organized in counterclockwise direction starting from front ([front, left, back, right])
    #             -- True = wall, False = no wall
    #             -- ie. [False, True, True, True]
    #         - list of potential neighbouring squares labels
    #     output: list of potential locations in maze (ie. ['A2', 'B2'])
    #     '''
    #     # set current wall configuration as None
    #     wall_config = None

    #     # print('get_location function - orientation: ', orientations)

    #     # True = wall in reading 
    #     if True in orientations:
    #         # sum of 1 = one wall in reading
    #         if sum(orientations) == 1:
    #             wall_config = 1
    #         # sum of 2 = 2 walls in reading
    #         elif sum(orientations) == 2:
    #             indices = [i for i, x in enumerate(orientations) if x == True]
    #             # if sum is even then walls are opposite
    #             if sum(indices) % 2 == 1:
    #                 wall_config = 2
    #             # if sum is odd then walls are adjacent
    #             else: 
    #                 wall_config = 3
    #         # sum of 3 = 3 walls in reading
    #         else:
    #             wall_config = 4
    #     # False = no walls in reading
    #     else:
    #         wall_config = 0

    #     # print('get_location function - wall_config: ', wall_config)

    #     # find all squares in map that have wall configuration
    #     self.prev_loc = [k for k, v in self.maze_map.items() if v == wall_config]

    #     # if neighbouring square is given and not in list, remove potential location
    #     if new_current_squares != None:
    #         print('NEW CURR SQUARE', new_current_squares)
    #         for square in new_current_squares[0]:
    #             for pot_loc in self.prev_loc:
    #                 if pot_loc not in self.neighbours(square):
    #                     print(square)
    #                     print(self.prev_loc)
    #                     self.prev_loc.remove(pot_loc)
    #         print('PREV', self.prev_loc)
    #         self.curr_loc = self.neighbours(self.prev_loc[0])[0]
    #         for square in self.curr_loc:
    #             if square not in new_current_squares[0]:
    #                 self.curr_loc.remove(square)
    #         return self.prev_loc, self.curr_loc
    #     else:
    #         return self.prev_loc

    # def neighbours(self, pot_loc):
    #     '''
    #     input: potential location label
    #     return: list of neighbouring squares, list of corresponding wall configurations
    #     '''
    #     # get coordinate of potential location
    #     for row in self.maze_labels:
    #         if pot_loc in row:
    #             y, x = self.maze_labels.index(row), row.index(pot_loc)

    #     # find neighbouring squares
    #     neighbouring_squares = [self.maze_labels[r][c] for r in range(y-1 if y > 0 else y, y + 2 if y < len(self.maze_labels)-1 else y + 1) for c in range(x-1 if x > 0 else x, x + 2 if x < len(self.maze_labels[0])-1 else x + 1)]
    #     # find neighbouring square wall configurations
    #     neighbouring_square_wall_configs = [self.wall_configs[r][c] for r in range(y-1 if y > 0 else y, y + 2 if y < len(self.wall_configs)-1 else y + 1) for c in range(x-1 if x > 0 else x, x + 2 if x < len(self.wall_configs[0])-1 else x + 1)]

    #     # find index of potential location in neighbouring squares
    #     pot_loc_index = neighbouring_squares.index(pot_loc)
        
    #     # remove potential location
    #     neighbouring_squares.pop(pot_loc_index)
    #     neighbouring_square_wall_configs.pop(pot_loc_index)
        
    #     return neighbouring_squares, neighbouring_square_wall_configs
    
    # def find_direction(self, current_loc, prev_loc):
    #     '''
    #     input: current location label and previous location label
    #     output: direction (N, E, S, W) that robot is facing
    #         - N is top of maze
    #     '''

    #     self.direction = None

    #     # compare y-axis lables
    #     # if y_curr < y_prev, robot is travelling N
    #     # if y_curr > y_prev, robot is travelling S
    #     # if y_curr = y_prev, robot is travelling E/W
    #     # if x_curr < x_prev, robot is travelling W
    #     # if x_curr > x_prev, robot is travelling E
    #     if ord(current_loc[0][0]) < ord(prev_loc[0][0]):
    #         self.direction = 'N'

    #     elif ord(current_loc[0]) > ord(prev_loc[0]):
    #         self.direction = 'N'
        
    #     else:
    #         if ord(current_loc[0][1]) < ord(prev_loc[0][1]):
    #             self.direction = 'W'

    #         else:
    #             self.direction = 'E'
                
    #     return self.direction


##############################################################################################

# # maze labels
# maze_labels = [[f'{y}{x}' for x in range(1, 9)] for y in ['A', 'B', 'C', 'D']]

# # maze wall configurations
# wall_configs = [
#     [2, 1, 3, 2, math.inf, 4, math.inf, 4],
#     [1, 2, math.inf, 2, 3, 0, 3, 1],
#     [3, math.inf, 4, math.inf, math.inf, 3, math.inf, 3],
#     [2, 3, 1, 3, 3, 2, math.inf, 4]
# ]

# # dict mapping maze labels with wall configurations
# maze_map = {}
# for l, c in zip(maze_labels, wall_configs):
#     for label, value in zip(l, c):
#         maze_map[label] = value 

### testing - NEED TO FINISH

Loc = mazeLocalization()

# # test 1
# test_wall_config1 = [False, True, True, True]
# test_wall_config2 = [False, True, False, True]

# current = Loc.get_location(test_wall_config1)
# print(current)
# for loc in current:
#     print(loc, Loc.neighbours(loc))
# front_square = Loc.get_location(test_wall_config2)
# print(front_square)
# current = Loc.get_location(test_wall_config1, new_current_squares=front_square)
# print(current) # should return 'D8'

# test 2
test_wall_config3 = [False, False, True, True]
test_wall_config4 = [False, True, False, True]

current = Loc.get_location(test_wall_config3)
print(current)
for loc in current:
    print(loc, Loc.neighbours(loc))
front_square = Loc.get_location(test_wall_config4)
print(front_square)
current = Loc.get_location(test_wall_config3, new_current_squares=front_square)
print(current) # should return 

# wall_configs = [
#     [2, 1, 3, 2, math.inf, 4, math.inf, 4],
#     [1, 2, math.inf, 2, 3, 0, 3, 1],
#     [3, math.inf, 4, math.inf, math.inf, 3, math.inf, 3],
#     [2, 3, 1, 3, 3, 2, math.inf, 4]
# ]

# wall_readings = []
# for row in wall_configs:
#     for column in row:
        
        
            


# for row in wall_configs:
#     for column in row:
#         if column != math.inf:
#             if column == 4:
#                 wall_readings = []
#             current = Loc.get_location(wall_readings)
#             while len(current) > 1:
#                 right_square = wall_configs[row][column+1]
                    