import time

from KIRB_python_arduino import PyArduino 
from KIRB_localization import MazeLocalization

PA = PyArduino(com_port="COM7")

PA.write(' w0-12')
time.sleep(3)
PA.write(' w0--12')
print('DONE')