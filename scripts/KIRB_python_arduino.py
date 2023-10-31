import serial
import time
from KIRB_obstacle_avoidance_v2 import ObstacleAvoidance
ser = serial.Serial('COM3', 9600, timeout=0) # Initialize COM port

OA = ObstacleAvoidance()
RUNNING = True

while RUNNING:
    cmd = input('enter: ')
    # cmd = 'u1'
    # writes command to Arduino
    ser.write(cmd.encode()) 
    time.sleep(3) 
    reading = ser.readline().strip().decode('ascii') 
    print(reading)
    
    sensor_readings = [float(r.split('=')[1]) for r in reading.split('|')[1:7]]
    print(sensor_readings)
    for i in range(len(sensor_readings)-1):
        OA.sensor_dict[OA.sensor_label_list[i]] = sensor_readings[i]
    
    print(OA.sensor_dict)

ser.close() # Closes connection