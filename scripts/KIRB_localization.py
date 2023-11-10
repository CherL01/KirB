import math
import numpy as np

class mazeLocalization():

    # initialize maze coordinates
    def __init__(self):
        
        # maze labels
        self.mazeLabels = [[f'{y}{x}' for x in range(1, 9)] for y in ['A', 'B', 'C', 'D']]
        
        # maze wall configurations
        self.wallConfigs = [
            [2, 1, 3, 2, math.inf, 4, math.inf, 4],
            [1, 2, math.inf, 2, 3, 0, 3, 1],
            [3, math.inf, 4, math.inf, math.inf, 3, math.inf, 3],
            [2, 3, 1, 3, 3, 2, math.inf, 4]
        ]

        # dict mapping maze labels with wall configurations
        self.mazeMap = {}
        for l, c in zip(self.mazeLabels, self.wallConfigs):
            for label, value in zip(l, c):
                self.mazeMap[label] = value 

    # get potential locations of current square
    def get_location(self, orientations, neighbouring_squares=None):
        '''
        input: 
            - list of wall readings at different orientations 
                -- organized in counterclockwise direction starting from front ([front, left, back, right])
                -- True = wall, False = no wall
                -- ie. [False, True, True, True]
            - list of neighbouring squares labels
        output: list of potential locations in maze (ie. ['A2', 'B2'])
        '''
        # set current wall configuration as None
        wall_config = None

        # print('get_location function - orientation: ', orientations)

        # True = wall in reading 
        if True in orientations:
            # sum of 1 = one wall in reading
            if sum(orientations) == 1:
                wall_config = 1
            # sum of 2 = 2 walls in reading
            elif sum(orientations) == 2:
                indices = [i for i, x in enumerate(orientations) if x == True]
                # if sum is even then walls are opposite
                if sum(indices) % 2 == 1:
                    wall_config = 2
                # if sum is odd then walls are adjacent
                else: 
                    wall_config = 3
            # sum of 3 = 3 walls in reading
            else:
                wall_config = 4
        # False = no walls in reading
        else:
            wall_config = 0

        # print('get_location function - wall_config: ', wall_config)

        # find all squares in map that have wall configuration
        self.current_location = [k for k, v in self.mazeMap.items() if v == wall_config]

        # if neighbouring square is given and not in list, remove potential location
        if neighbouring_squares != None:
            for square in neighbouring_squares[0]:
                for pot_loc in self.current_location:
                    if square not in self.neighbours(pot_loc):
                        self.current_location.remove(pot_loc)

        return self.current_location

    def neighbours(self, pot_loc):
        '''
        input: potential location label
        return: list of neighbouring squares, list of corresponding wall configurations
        '''
        # get coordinate of potential location
        for row in self.mazeLabels:
            if pot_loc in row:
                y, x = self.mazeLabels.index(row), row.index(pot_loc)

        # find neighbouring squares
        neighbouring_squares = [self.mazeLabels[r][c] for r in range(y-1 if y > 0 else y, y + 2 if y < len(self.mazeLabels)-1 else y + 1) for c in range(x-1 if x > 0 else x, x + 2 if x < len(self.mazeLabels[0])-1 else x + 1)]
        # find neighbouring square wall configurations
        neighbouring_square_wall_configs = [self.wallConfigs[r][c] for r in range(y-1 if y > 0 else y, y + 2 if y < len(self.wallConfigs)-1 else y + 1) for c in range(x-1 if x > 0 else x, x + 2 if x < len(self.wallConfigs[0])-1 else x + 1)]

        # find index of potential location in neighbouring squares
        pot_loc_index = neighbouring_squares.index(pot_loc)
        
        # remove potential location
        neighbouring_squares.pop(pot_loc_index)
        neighbouring_square_wall_configs.pop(pot_loc_index)
        
        return neighbouring_squares, neighbouring_square_wall_configs

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

# test 1
current_test_wall_config1 = [False, True, True, True]
front_test_wall_config1 = [False, True, False, True]

current = Loc.get_location(current_test_wall_config1)
print(current)
for loc in current:
    print(loc, Loc.neighbours(loc))
front_square = Loc.get_location(front_test_wall_config1)
print(front_square)
current = Loc.get_location(current_test_wall_config1, neighbouring_squares=front_square)
print(current) # should return 'D8'

# # test 2
# current_test_wall_config2 = [True, True, False, False]
# back_test_wall_config2 = [False, True, False, True]

# current = Loc.get_location(current_test_wall_config2)
# print(current)
# for loc in current:
#     print(loc, Loc.neighbours(loc))
# back_square = Loc.get_location(back_test_wall_config2)
# print(back_square)
# current = Loc.get_location(current_test_wall_config2, neighbouring_squares=back_square)
# print(current) # should return 'B4'

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
                    