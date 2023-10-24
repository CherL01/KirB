'''
This file is part of SimMeR, an educational mechatronics robotics simulator.
Initial development funded by the University of Toronto MIE Department.
Copyright (C) 2023  Ian G. Bennett

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
'''

# Basic echo client, for testing purposes
# Code modified from examples on https://realpython.com/python-sockets/
# and https://www.geeksforgeeks.org/python-display-text-to-pygame-window/

import socket
import struct
import time
import math
from threading import Thread
import _thread
from datetime import datetime

def transmit(data):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('utf-8'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
            _thread.interrupt_main()
        except TimeoutError:
            print('Tx socket timed out.')
            _thread.interrupt_main()
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')
            _thread.interrupt_main()

def receive():
    global responses
    global time_rx
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
            try:
                s2.connect((HOST, PORT_RX))
                response_raw = s2.recv(1024)
                if response_raw:
                    responses = bytes_to_list(response_raw)
                    time_rx = datetime.now().strftime("%H:%M:%S")
            except (ConnectionRefusedError, ConnectionResetError):
                print('Rx connection was refused or reset.')
                _thread.interrupt_main()
            except TimeoutError:
                print('Response not received from robot.')
                _thread.interrupt_main()

def bytes_to_list(msg):
    num_responses = int(len(msg)/8)
    data = struct.unpack("%sd" % str(num_responses), msg)
    return data


### Network Setup ###
HOST = '127.0.0.1'  # The server's hostname or IP address
PORT_TX = 61200     # The port used by the *CLIENT* to receive
PORT_RX = 61201     # The port used by the *CLIENT* to send data

# Received responses
responses = [False]
time_rx = 'Never'

# Create tx and rx threads
Thread(target = receive, daemon = True).start()

# Run the sequence of commands
RUNNING = True
# cmd_sequence = ['w0-36', 'r0-80', 'w0-34', 'r0-80', 'w0-11', 'r0--90', 'w0-25', 'r0--95', 'w0-6', 'r0-720']

frontSensor = 0
topRSensor = 0
topLSensor = 0
sensorDifference = 0
sensorDifferenceLimit = 1

ct = 0
while RUNNING:

    # move forward 1 inches
    #transmit('w0-1')
    #time.sleep(0.1)
    
    # check front sensor
    transmit('u0')
    time.sleep(0.1)
    print(f"Ultrasonic FRONT reading: {round(responses[0], 3)}")
    frontSensor = responses[0]
    
    if responses[0] < 2:
        RUNNING = False
        print("Emergency stop!")
        transmit('xx')
        break
    
    # check top left sensor
    transmit('u1')
    time.sleep(0.1)
    print(f"Ultrasonic TOP-LEFT reading: {round(responses[0], 3)}")
    topLSensor = responses[0]
    
    if responses[0] < 2:
        RUNNING = False
        print("Emergency stop!")
        transmit('xx')
        break

    # check top right sensor
    transmit('u2')
    time.sleep(0.1)
    print(f"Ultrasonic TOP-RIGHT reading: {round(responses[0], 3)}")
    topRSensor = responses[0]
    
    if responses[0] < 2:
        RUNNING = False
        print("Emergency stop!")
        transmit('xx')
        break    
    
    # find difference between top right and top left sensors
    sensorDifference = abs(topLSensor-topRSensor)
    if sensorDifference > sensorDifferenceLimit:
        
        if topLSensor > topRSensor:
            transmit('r0--5')
            time.sleep(0.1)
            print("(<---) Turned slightly left...")
            
            if topLSensor > 4 and topRSensor > 4 and frontSensor > 4:
                # move forward 1 inch
                transmit('w0-1.2')
                time.sleep(0.1)
            
        elif topLSensor < topRSensor:
            transmit('r0-5')
            time.sleep(0.1)
            print("(--->) Turned slightly right...")
            
            if topLSensor > 4 and topRSensor > 4 and frontSensor > 4:
                # move forward 1 inch
                transmit('w0-1.2')
                time.sleep(0.1)
            
    elif frontSensor > 4:
        # move forward 1 inch
        transmit('w0-1')
        time.sleep(0.1)
    
    elif frontSensor < 4:
        #rotate 180
        transmit('r0-180')
        time.sleep(0.1)
        print("(<--- U-TURN!! --->)")
    
    if responses[0] < 2:
        RUNNING = False
        print("Emergency stop!")
        transmit('xx')
        break 
    
    # # check left sensor
    # transmit('u3')
    # time.sleep(0.1)
    # print(f"Ultrasonic LEFT reading: {round(responses[0], 3)}")
    
    # # check right sensor
    # transmit('u4')
    # time.sleep(0.1)
    # print(f"Ultrasonic RIGHT reading: {round(responses[0], 3)}")
    
    print("--------------")
    
    time.sleep(0.1)
    ct += 1
    
    if ct > 1000:
        RUNNING = False
        print("Sequence Complete!")
    
