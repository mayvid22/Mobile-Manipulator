# import rclpy
# from rclpy.node import Node
# from serial_motor_demo_msgs.msg import MotorCommand
# from serial_motor_demo_msgs.msg import MotorVels
# from serial_motor_demo_msgs.msg import EncoderVals
# import time
# import math
# import serial
# from threading import Lock
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry


# class MotorDriver(Node):

#     def __init__(self):
#         super().__init__('motor_driver')


#         # Setup parameters

#         # self.declare_parameter('encoder_cpr', value=0)
#         # if (self.get_parameter('encoder_cpr').value == 0):
#         #     print("WARNING! ENCODER CPR SET TO 0!!")


#         # self.declare_parameter('loop_rate', value=0)
#         # if (self.get_parameter('loop_rate').value == 0):
#         #     print("WARNING! LOOP RATE SET TO 0!!")


#         self.declare_parameter('serial_port', value="/dev/ttyUSB0")
#         self.serial_port = self.get_parameter('serial_port').value


#         self.declare_parameter('baud_rate', value=57600)
#         self.baud_rate = self.get_parameter('baud_rate').value


#         self.declare_parameter('serial_debug', value=False)
#         self.debug_serial_cmds = self.get_parameter('serial_debug').value
#         if (self.debug_serial_cmds):
#             print("Serial debug enabled")

#         # Setup topics & services

#         self.subscription = self.create_subscription(
#             MotorCommand,
#             'motor_command',
#             self.motor_command_callback,
#             10)

#         self.speed_pub = self.create_publisher(MotorVels, 'motor_vels', 10)

#         # self.encoder_pub = self.create_publisher(EncoderVals, 'encoder_vals', 10)

#         # Member Variables

#         self.last_enc_read_time = time.time()
#         self.last_m1_enc = 0
#         self.last_m2_enc = 0
#         self.m1_spd = 0.0
#         self.m2_spd = 0.0

#         self.mutex = Lock()


#         # Open serial comms

#         print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
#         self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
#         print(f"Connected to {self.conn}")
        

        


#     # Raw serial commands
    
#     def send_pwm_motor_command(self, mot_1_pwm, mot_2_pwm):
#         self.send_command(f"o {int(mot_1_pwm)} {int(mot_2_pwm)}")

#     # def send_feedback_motor_command(self, mot_1_ct_per_loop, mot_2_ct_per_loop):
#     #     self.send_command(f"m {int(mot_1_ct_per_loop)} {int(mot_2_ct_per_loop)}")

#     # def send_encoder_read_command(self):
#     #     resp = self.send_command(f"e")
#     #     if resp:
#     #         return [int(raw_enc) for raw_enc in resp.split()]
#     #     return []


#     # More user-friendly functions

#     def motor_command_callback(self, motor_command):
#         if (motor_command.is_pwm):
#             self.send_pwm_motor_command(motor_command.mot_1_req_rad_sec, motor_command.mot_2_req_rad_sec)
        

#     # def check_encoders(self):
#     #     new_time = time.time()
#     #     time_diff = new_time - self.last_enc_read_time
#     #     self.last_enc_read_time = new_time

#         # m1_diff = resp[0] - self.last_m1_enc
#         # self.last_m1_enc = resp[0]
#         # m2_diff = resp[1] - self.last_m2_enc
#         # self.last_m2_enc = resp[1]

#         # rads_per_ct = 2*math.pi/self.get_parameter('encoder_cpr').value
#         # self.m1_spd = m1_diff*rads_per_ct/time_diff
#         # self.m2_spd = m2_diff*rads_per_ct/time_diff

#         # spd_msg = MotorVels()
#         # spd_msg.mot_1_rad_sec = self.m1_spd
#         # spd_msg.mot_2_rad_sec = self.m2_spd
#         # self.speed_pub.publish(spd_msg)

#         # enc_msg = EncoderVals()
#         # enc_msg.mot_1_enc_val = self.last_m1_enc
#         # enc_msg.mot_2_enc_val = self.last_m2_enc
#         # self.encoder_pub.publish(enc_msg)

#     # Utility functions

#     def send_command(self, cmd_string):
        
#         self.mutex.acquire()
#         try:
#             cmd_string += "\r"
#             self.conn.write(cmd_string.encode("utf-8"))
#             if (self.debug_serial_cmds):
#                 print("Sent: " + cmd_string)

#             ## Adapted from original
#             c = ''
#             value = ''
#             while c != '\r':
#                 c = self.conn.read(1).decode("utf-8")
#                 if (c == ''):
#                     print("Error: Serial timeout on command: " + cmd_string)
#                     return ''
#                 value += c

#             value = value.strip('\r')

#             if (self.debug_serial_cmds):
#                 print("Received: " + value)
#             return value
#         finally:
#             self.mutex.release()

#     def close_conn(self):
#         self.conn.close()



# def main(args=None):
    
#     rclpy.init(args=args)

#     motor_driver = MotorDriver()

#     rate = motor_driver.create_rate(2)
#     while rclpy.ok():
#         rclpy.spin_once(motor_driver)
#         # motor_driver.check_encoders()


#     motor_driver.close_conn()
#     motor_driver.destroy_node()
#     rclpy.shutdown()


import rclpy
from rclpy.node import Node
from serial_motor_demo_msgs.msg import MotorCommand
from geometry_msgs.msg import Twist

import time
import math
import serial
from threading import Lock
import numpy as np

class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        self.declare_parameter('serial_port', value="/dev/ttyUSB0")
        self.serial_port = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=57600)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if self.debug_serial_cmds:
            print("Serial debug enabled")

        self.declare_parameter('wheel_base', value=0.2)  # Distance between wheels in meters
        self.declare_parameter('wheel_radius', value=0.05)  # Radius of the wheels in meters

        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        self.motor_command_subscription = self.create_subscription(
            MotorCommand,
            'motor_command',
            self.motor_command_callback,
            10)

        self.speed_pub = self.create_publisher(MotorCommand, 'motor_command', 10)
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel_converted', 10)

        self.mutex = Lock()

        # Open serial comms
        print(f"Connecting to port {self.serial_port} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        print(f"Connected to {self.conn}")

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate wheel speeds
        left_wheel_speed = (linear_x - (angular_z * self.wheel_base / 2)) / self.wheel_radius
        right_wheel_speed = (linear_x + (angular_z * self.wheel_base / 2)) / self.wheel_radius

        # Type cast to float32
        left_wheel_speed = np.float32(left_wheel_speed)
        right_wheel_speed = np.float32(right_wheel_speed)

        self.send_wheel_commands(left_wheel_speed, right_wheel_speed)

    def motor_command_callback(self, motor_command):
        left_wheel_speed = motor_command.mot_1_req_rad_sec
        right_wheel_speed = motor_command.mot_2_req_rad_sec

        # Calculate linear and angular velocities
        linear_x = self.wheel_radius * (left_wheel_speed + right_wheel_speed) / 2
        angular_z = self.wheel_radius * (right_wheel_speed - left_wheel_speed) / self.wheel_base

        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z

        self.twist_pub.publish(twist_msg)

    def send_wheel_commands(self, left_wheel_speed, right_wheel_speed):
        msg = MotorCommand()
        msg.is_pwm = False  # Assuming you are using feedback mode for speed control
        msg.mot_1_req_rad_sec = left_wheel_speed
        msg.mot_2_req_rad_sec = right_wheel_speed
        self.speed_pub.publish(msg)

    def send_command(self, cmd_string):
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))
            if self.debug_serial_cmds:
                print("Sent: " + cmd_string)

            c = ''
            value = ''
            while c != '\r':
                c = self.conn.read(1).decode("utf-8")
                if c == '':
                    print("Error: Serial timeout on command: " + cmd_string)
                    return ''
                value += c

            value = value.strip('\r')

            if self.debug_serial_cmds:
                print("Received: " + value)
            return value
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()

def main(args=None):
    rclpy.init(args=args)

    motor_driver = MotorDriver()

    rate = motor_driver.create_rate(2)
    while rclpy.ok():
        rclpy.spin_once(motor_driver)

    motor_driver.close_conn()
    motor_driver.destroy_node()
    rclpy.shutdown()
