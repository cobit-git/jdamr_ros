#!/usr/bin/env python3
#from JDamr_lib import JDamr
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, LaserScan
import rospy
import random
import serial
import threading 
import struct
import time 


'''
In this script, we study follwings:
1. How to write basic ROS node for robot car
2. This node subscribes teleop node using "cmd_vel" topic.
3. This node control robot through serial according to teleop input. 
'''

class JDamr(object):
    def __init__(self, com="/dev/ttyACM0"):
        self.ser = serial.Serial(com, 115200)
        self.HEAD = 0xf5
        self.CMD_SET_MOTOR = 0x01
        self.CMD_GET_SPEED = 0x02
        self.CMD_GET_ENCODER = 0x03
        self.CMD_CAR_RUN = 0x04

        self.encoder1 = 0
        self.encoder2 = 0
        self.encoder3 = 0
        self.encoder4 = 0

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
            taks_name = "serial_thread"
            rx_task = threading.Thread(target=self.receive_data, name=taks_name)
            rx_task.setDaemon(True)
            rx_task.start()
            print("Start serial receive thread ")
            time.sleep(.05)
        except:
            pass

    def parse_cmd(self, payload):
        if self.CMD_GET_ENCODER == payload[0]:
            print(payload)
            encode1_str = payload[1:5]
            encode2_str = payload[5:9]
            encode3_str = payload[9:13]
            encode4_str = payload[13:17]
            self.encode1 = int.from_bytes(encode1_str, byteorder="big")
            print(self.encode1)
            self.encode2 = int.from_bytes(encode2_str, byteorder="big")
            print(self.encode2)
            self.encode3 = int.from_bytes(encode3_str, byteorder="big")
            print(self.encode3)
            self.encode4 = int.from_bytes(encode4_str, byteorder="big")
            print(self.encode4)

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
    
    '''
    drive_mode: 
    1: go foreward 
    2: go backward 
    3: turn left 
    4: turn right 
    speed: 1 ~ 100 
    '''
    def set_car_run(self, drive_mode, speed):
        try:
            speed_0 = bytearray(struct.pack('b', speed))
            drive_mode_0 = bytearray(struct.pack('b', drive_mode))
            cmd = [self.HEAD, 0x00, self.CMD_CAR_RUN, drive_mode_0[0], speed_0[0]]
            cmd[1] = len(cmd) - 1
            checksum = 0xff #sum(cmd) & 0xff
            cmd.append(checksum)
            self.ser.write(cmd)
            print("car_run:", cmd)
            time.sleep(0.1)
        except:
            print('---set_car_run error!---')
            pass

class jdamr_driver: 
    def __init__(self):
        rospy.on_shutdown(self.reset_amr)
        self.jdamr = JDamr()
        # This variable hold prefixs for joint state 
        self.Prefix = rospy.get_param("~prefix", "")
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, queue_size=100 )
        # This code generate "joint state" token publisher 
        self.states_pub = rospy.Publisher('joint_states', JointState, queue_size=100)
        time.sleep(1)
        self.jdamr.receive_thread()
        # wheel joint state 
        self.wheel_state = JointState()
        self.motor_speed = 0 # 0 : motor stop

    def reset_amr(self):
        pass
        #self.cmd_vel_sub.unregister()
        #self.states_pub.unregister()

    '''
    This function publish joint state of wheels. Using joint state, we can control wheels on rviz 
    
    '''    
    i = -3.14
    def pub_data(self):
        '''
        In this step, we add only joinst state. 
        '''

        i = -3.14
        while not rospy.is_shutdown():
            # preparing joint state 
            self.wheel_state.header.stamp = rospy.Time.now()
            self.wheel_state.header.frame_id = 'joint_states'
            if len(self.Prefix)==0:
                self.wheel_state.name = ["wheel1_joint", "wheel2_joint",
                              "wheel3_joint", "wheel4_joint"]
            else:
                self.wheel_state.name = [self.Prefix+"/wheel1_joint", self.Prefix+"/wheel2_joint",
                              self.Prefix+"/wheel3_joint", self.Prefix+"/wheel4_joint"]
           


            i += 0.001*self.motor_speed # Control of wheel speed on rviz
            if i > 3.14:
                i = -3.14
            elif i < -3.14:
                i = 3.14
            self.wheel_state.position = [i, 0, 0, 0]
            self.states_pub.publish(self.wheel_state)


    def cmd_vel_callback(self, msg):
        if not isinstance(msg, Twist):
            return 
        x = msg.linear.x
        y = msg.linear.y
        angle = msg.angular.z
        rospy.loginfo("cmd_velx: {}, cmd_vely: {}, cmd_ang: {}".format(x, y, angle))
        # use 'u' and 'i' key to control motor. 'angle' value is changed.
        self.motor_speed = angle * 100
        # limiting max motor value 
        if self.motor_speed > 100:
            self.motor_speed = 100
        elif self.motor_speed < -100:
            self.motor_speed = -100
        print('uga: ', self.motor_speed)
        # according to teleop key input, it control real motor 
        self.jdamr.set_motor(int(self.motor_speed), 0, 0, 0)   
        
if __name__ == '__main__':
    rospy.init_node("jdamr_driver_node", anonymous=False)
    rate = rospy.Rate(10) # 10hz
    driver = jdamr_driver()
    driver.pub_data()
    rospy.spin()
    