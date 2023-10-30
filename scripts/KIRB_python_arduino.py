import serial
import time
ser = serial.Serial('COM3', 9600, timeout=0) # Initialize COM port
s = ' '
while s:
    s = input('enter: ')
    # s=' ee'
    # print(type(s))
    ser.write(s.encode()) # writes letter to Arduino
    time.sleep(3) # you can also use pause(0.1)
    print('python command')
    print(ser.readline().strip().decode('ascii')) # reads from Arduino
    print('---------------------------')
ser.close() # Closes connection