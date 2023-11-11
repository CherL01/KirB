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
    def get_location(self, orientations, new_current_squares=None):
        '''
        input: 
            - list of wall readings at different orientations 
                -- organized in counterclockwise direction starting from front ([front, left, back, right])
                -- True = wall, False = no wall
                -- ie. [False, True, True, True]
            - list of potential neighbouring squares labels
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
        self.prev_loc = [k for k, v in self.mazeMap.items() if v == wall_config]

        # if neighbouring square is given and not in list, remove potential location
        if new_current_squares != None:
            print('NEW CURR SQUARE', new_current_squares)
            for square in new_current_squares[0]:
                for pot_loc in self.prev_loc:
                    if pot_loc not in self.neighbours(square):
                        print(square)
                        print(self.prev_loc)
                        self.prev_loc.remove(pot_loc)
            print('PREV', self.prev_loc)
            self.curr_loc = self.neighbours(self.prev_loc[0])[0]
            for square in self.curr_loc:
                if square not in new_current_squares[0]:
                    self.curr_loc.remove(square)
            return self.prev_loc, self.curr_loc
        else:
            return self.prev_loc

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
    
    def find_direction(self, current_loc, prev_loc):
        '''
        input: current location label and previous location label
        output: direction (N, E, S, W) that robot is facing
            - N is top of maze
        '''

        self.direction = None

        # compare y-axis lables
        # if y_curr < y_prev, robot is travelling N
        # if y_curr > y_prev, robot is travelling S
        # if y_curr = y_prev, robot is travelling E/W
        # if x_curr < x_prev, robot is travelling W
        # if x_curr > x_prev, robot is travelling E
        if ord(current_loc[0][0]) < ord(prev_loc[0][0]):
            self.direction = 'N'

        elif ord(current_loc[0]) > ord(prev_loc[0]):
            self.direction = 'N'
        
        else:
            if ord(current_loc[0][1]) < ord(prev_loc[0][1]):
                self.direction = 'W'

            else:
                self.direction = 'E'
                
        return self.direction

        
    
    # def bfs(self, des_loc):
    #     '''
    #     input: current location label and direction
    #     output: shortest path to desired location
    #     '''

    #     # Maintain a queue of paths
    #     queue = []

    #     # Push the first path into the queue
    #     queue.append([self.current_location])
    #     while queue:

    #         # Get the first path from the queue
    #         path = queue.pop(0)

    #         # Get the last node from the path
    #         node = path[-1]

    #         # Path found
    #         if node == end:
    #             return path

    #         # Enumerate all adjacent nodes, construct a new path and push it into the queue
    #         for adjacent in graph.get(node, []):
    #             new_path = list(path)
    #             new_path.append(adjacent)
    #             queue.append(new_path)

    #     print bfs(graph, '1', '11')


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
                    