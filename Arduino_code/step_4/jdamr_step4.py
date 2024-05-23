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
        if self.CMD_GET_ENCODER == payload[0]:
            encode1_str = payload[1:5]
            print(encode1_str)
            encode1 =int.from_bytes(encode1_str, byteorder="big")
            print(encode1)

    '''
    step 2 subject 
    - Sending command packet to run motor 
    - protocol 
      - Header byte 
      - length byte
      - command byte
      - motor 1 speed byte
      - motor 2 speed byte
      - motor 3 speed byte
      - motor 4 speed byte   
      - checksum byte - tempral 0xff 
    
    '''
    def set_motor(self, speed_1, speed_2, speed_3, speed_4):
        try:
            speed_a = bytearray(struct.pack('b', speed_1))
            speed_b = bytearray(struct.pack('b', speed_2))
            speed_c = bytearray(struct.pack('b', speed_3))
            speed_d = bytearray(struct.pack('b', speed_4))
            cmd = [self.HEAD, 0x00, self.CMD_SET_MOTOR,
                    speed_a[0], speed_b[0], speed_c[0], speed_d[0]]
            cmd[1] = len(cmd) - 1
            checksum = 0xff #sum(cmd) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            print("motor:", cmd)
            time.sleep(0.1)
        except:
            print("set_motor error")
            pass

if __name__ == '__main__':
    com = '/dev/ttyACM0'
    bot = JDamr(com)
    time.sleep(1)
    bot.receive_thread()
   
    while True:
       '''
       Step2 
       '''
       bot.set_motor(100, 100, 100, 100)
       time.sleep(2)
       bot.set_motor(0, 0, 0, 0)
       time.sleep(2)



