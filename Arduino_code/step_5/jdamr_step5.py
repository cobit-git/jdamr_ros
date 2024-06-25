import serial
import threading 
import struct
import time 

class JDamr(object):
    def __init__(self, com="/dev/ttyACM0"):
        self.ser = serial.Serial(com, 115200)
        self.HEAD = 0xf5
        self.CMD_SET_MOTOR = 0x01
        self.CMD_GET_SPEED = 0x02
        self.CMD_GET_ENCODER = 0x03
        self.CMD_CAR_RUN = 0x04
        self.CMD_GET_IMU = 0x05

        if self.ser.isOpen():
            print("JDamr serial port opened!")
        else:
            print("Can't open JDamr serial port!")
        time.sleep(1)
    
    '''
    Protocol 
    - Packets have following bytes.
      - Header byte 
      - length byte
      - command byte
      - payload bytes 
      - checksum byte 
    '''
    def receive_data(self):     
        self.ser.flushInput()
        while True:
            head = bytearray(self.ser.read())[0]
            if head == self.HEAD:
                length = bytearray(self.ser.read())[0]  
                payload = [] 
                for i in range(length-1):
                    value = bytearray(self.ser.read())[0]
                    payload.append(value)
                self.parse_cmd(payload)

    def receive_thread(self):
        try:
            task_name = "serial_thread"
            rx_task = threading.Thread(target=self.receive_data, name=task_name)
            rx_task.setDaemon(True)
            rx_task.start()
            print("Start serial receive thread ")
            time.sleep(.05)
        except:
            pass

    def parse_cmd(self, payload):
        if self.CMD_GET_IMU == payload[0]:
            pitch_str = payload[1:5]
            print(pitch_str)
            pitch = int.from_bytes(pitch_str, byteorder="big", signed=True)
            print(pitch)

   
if __name__ == '__main__':
    com = '/dev/ttyACM0'
    bot = JDamr(com)
    time.sleep(1)
    bot.receive_thread()
   
    while True:
        pass
