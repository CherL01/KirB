import serial
import time
# from KIRB_obstacle_avoidance_v2 import ObstacleAvoidance
ser = serial.Serial('COM5', 9600, timeout=0) # Initialize COM port

# OA = ObstacleAvoidance()
RUNNING = True
run_count = 0

while RUNNING:
    
    if run_count == 0:
    # cmd = input('enter: ')
        cmd = ' r0-1'
        print(cmd)
        # writes command to Arduino
        ser.write(cmd.encode())
        time.sleep(4) 
        reading = ser.readline().strip().decode('ascii') 
        print(reading)
        
        sensor_readings = [float(r.split('=')[1]) for r in reading.split('|')[1:7]]
        print(sensor_readings)
    if run_count > 0:
        cmd = ' w0-1'
        print(cmd)
        # writes command to Arduino
        ser.write(cmd.encode()) 
        time.sleep(5) 
        reading = ser.readline().strip().decode('ascii') 
        print(reading)
        
        sensor_readings = [float(r.split('=')[1]) for r in reading.split('|')[1:7]]
        print(sensor_readings)
    # for i in range(len(sensor_readings)-1):
    #     OA.sensor_dict[OA.sensor_label_list[i]] = sensor_readings[i]
    
    # print(OA.sensor_dict)
    
    run_count += 1
    if run_count == 3:
        RUNNING = False

ser.close() # Closes connection