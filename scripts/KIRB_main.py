from KIRB_obstacle_avoidance import ObstacleAvoidance
from KIRB_localization import MazeLocalization
from KIRB_python_arduino import PyArduino

### define generalized movement commands to feed into obstacle avoidance
# example:
# right_turn = ['r0-90']
# move_right = right_turn + ['w0-12']
# or put in a dictionary etc.