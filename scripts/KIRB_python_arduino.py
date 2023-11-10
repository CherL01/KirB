import serial
import time

class PyArduino:
    def __init__(self, com_port, baud_rate = 9600, timeout = 0.1, keep_all = True, mode = 'FIFO', msg_delim = "\n") -> None:
        ''' A class that creates a non blocking read write interface with the arduino

        com_port -> str:
            String for the COM port, example "COM9"
        baud_rate -> int:
            (default 9600) Explained in PySerial
        timeout -> float:
            (default 0.1) Timeout for if serial object is used for readlines
        keep_all -> bool:
            (default True) Whether to save stuff in all_msgs_rec and all_msgs_sent
        mode -> str:
            (default FIFO) Mode for receiving messages. Choose from FIFO and LIFO
        msg_delim -> str:
            (default "\n") delimiter for end of message
        '''
        self.msg_buffer = ""
        self.msgs = []
        self.all_msgs_rec = []
        self.all_msgs_sent = []
        self.keep_all = keep_all
        self.mode = mode
        self.msg_delim = msg_delim
        # initialize serial connection with the arduino, 3 second timeout to make sure nothing breaks
        self.ser = serial.Serial(com_port, baud_rate, timeout=timeout)
        time.sleep(3)
        

    def write(self, msg):
        ''' writes msg to serial port, keeps messages in all_msgs_sent if keep_all = True '''
        self.ser.write(bytes(msg, 'ascii'))
        if self.keep_all:
            self.all_msgs_sent.append(msg)

    def pop_read(self):
        ''' Reads messages from serial port if there is a reading, keeps all messages in all_msg_rec'''
        if (self.ser.in_waiting > 0):
            data = self.ser.read(self.ser.in_waiting).decode('ascii')
            self.msg_buffer = self.msg_buffer + data
            if self.msg_delim in self.msg_buffer:
                split_buf = self.msg_buffer.split(self.msg_delim)
                msg_rec = split_buf.pop(0)
                self.msg_buffer = self.msg_delim.join(split_buf)

                self.msgs.append(msg_rec)
                self.all_msgs_rec.append(msg_rec)

        if self.msgs:
            if self.mode == "FIFO":
                return self.msgs.pop(0)
            elif self.mode == "LIFO":
                return self.msgs.pop(-1)
            else:
                raise ValueError(f"MODE '{self.mode}' NOT SUPPORTED")
        return None
    
    def blocking_read(self):
        '''Blocking Read for testing'''
        while True:
            reading = self.pop_read()
            if reading is not None:
                return reading

if __name__ == "__main__":
    arduino = PyArduino(com_port="COM9")
    while True:
        inp = input("Give a command via keyboard\n")
        start = time.time()
        arduino.write(inp)
        print("Waiting for Arduino Output:")
        print(arduino.blocking_read())
        print("Output finished")
        print(f"Time taken: {time.time() - start}")

# from KIRB_ob"stacle_avoidance_v2 import ObstacleAvoidance

# # OA = ObstacleAvoidance()
# RUNNING = True
# run_count = 0

# while RUNNING:
    
#     if run_count == 0:
#     # cmd = input('enter: ')
#         cmd = ' r0-1'
#         print(cmd)
#         # writes command to Arduino
#         ser.write(cmd.encode())
#         time.sleep(4) 
#         reading = ser.readline().strip().decode('ascii') 
#         print(reading)
        
#         sensor_readings = [float(r.split('=')[1]) for r in reading.split('|')[1:7]]
#         print(sensor_readings)
#     if run_count > 0:
#         cmd = ' w0-1'
#         print(cmd)
#         # writes command to Arduino
#         ser.write(cmd.encode()) 
#         time.sleep(5) 
#         reading = ser.readline().strip().decode('ascii') 
#         print(reading)
        
#         sensor_readings = [float(r.split('=')[1]) for r in reading.split('|')[1:7]]
#         print(sensor_readings)
#     # for i in range(len(sensor_readings)-1):
#     #     OA.sensor_dict[OA.sensor_label_list[i]] = sensor_readings[i]
    
#     # print(OA.sensor_dict)
    
#     run_count += 1
#     if run_count == 3:
#         RUNNING = False

# ser.close() # Closes connection



