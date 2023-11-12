import serial
import time
SER = serial.Serial('COM10', 9600, timeout=0)

while True:
    s = input('Type something: ')
    print(s)
    SER.write(s.encode())
    time.sleep(3)
    print(SER.readline().strip().decode('ascii'))

SER.close()
    