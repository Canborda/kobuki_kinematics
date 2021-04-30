#!/usr/bin/python3

import rospy
from pynput.keyboard import Key, Listener

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import numpy as np
from rospy.numpy_msg import numpy_msg

current_key = None

# Key pressed and released Callbacks

def pressed(key):
    global current_key
    try:
        current_key = key.char
    except AttributeError:
        current_key = key

def released(key):
    global current_key
    current_key = None


class velocity_publisher:

    def __init__(self):
        
        # Robot model parameters

        r = 0.076/2
        l = 0.23/2

        left_alpha = np.pi/2
        left_betha = 0.0

        right_alpha = -np.pi/2
        right_betha = np.pi

        # Jacobian
        
        J1 = np.array([[np.sin(left_alpha+left_betha), -np.cos(left_alpha+left_betha), -l*np.cos(left_betha)],
                       [np.sin(right_alpha+right_betha), -np.cos(right_alpha+right_betha), -l*np.cos(right_betha)]])

        J2 = r*np.identity(2)

        self.Jacobian = np.matmul(np.linalg.pinv(J2), J1)

        print('J1:')
        print(J1)

        print('J2:')
        print(J2)

        # Subscriber
        self.cmd_vel_subscriber = rospy.Subscriber("cmd_vel", Twist, self.cmd_vel_cb, queue_size=10)

        # Publisher
        self.pub_left_wheel = rospy.Publisher("/left_wheel_ctrl/command", Float64, queue_size=10)
        self.pub_right_wheel = rospy.Publisher("/right_wheel_ctrl/command", Float64, queue_size=10)

        # LISTENER

        # setup keyboard listener
        self.listener = Listener(on_press=pressed, on_release=released)
        self.listener.start()
        rospy.loginfo('>> STATUS: Init.')

        # Polling

        self.msg = Twist()
        self.rate = rospy.Rate(10) # publish messages at 10Hz
        
        while not rospy.is_shutdown():

            # Exit

            if current_key == Key.esc:
                rospy.loginfo('>> STATUS: Quit.')
                rospy.signal_shutdown('Request shutdown')
            
            # Move X
            
            elif current_key == 'w':
                self.msg.linear.x = self.msg.linear.x+0.02 if self.msg.linear.x < 1.0 else 1.0

            elif current_key == 's':
                self.msg.linear.x = self.msg.linear.x-0.02 if self.msg.linear.x > -1.0 else -1.0
            
            # Rotate

            elif current_key == 'a':
                self.msg.angular.z = self.msg.angular.z+10 if self.msg.angular.z < 180 else 180
            
            elif current_key == 'd':
                self.msg.angular.z = self.msg.angular.z-10 if self.msg.angular.z > -180 else -180
            
            # Stop

            elif current_key == 'x':
                self.msg.linear.x = 0.0
                self.msg.angular.z = 0.0

            # Inverse kinematics

            result = np.matmul(self.Jacobian, [self.msg.linear.x, self.msg.linear.y, self.msg.angular.z*np.pi/180])
            
            # Send velocity

            msgFloat = Float64()
            msgFloat.data = result[0]
            self.pub_left_wheel.publish(msgFloat)

            msgFloat = Float64()
            msgFloat.data = result[1]
            self.pub_right_wheel.publish(msgFloat)

            self.rate.sleep()

            # Printing

            print(chr(27)+"[2J")
            print('-'*70)
            print('For Vx++ use w \t\t For ω++ use a \t\t For STOP use x')
            print('For Vx-- use s \t\t For ω-- use d \t\t Press ESC to quit')
            print('-'*70)
            print('')
            print('\tVx = {:.2} m/s\t\t Left wheel = {:.4} rpm'.format(self.msg.linear.x, result[0]*30/np.pi))
            print('\tVy = {:.2} m/s\t\tRight wheel = {:.4} rpm'.format(self.msg.linear.y, result[1]*30/np.pi))
            print('\t ω = {} deg/s'.format(self.msg.angular.z))
            print('')
            print('-'*70)
            print('')
            print('Key pressed = ', current_key)


    def cmd_vel_cb(self, cmd_vel):

        command = np.array([0,0,0], dtype=np.float)
        command[0] = cmd_vel.linear.x
        command[1] = cmd_vel.linear.y
        command[2] = cmd_vel.angular.z

        print('Command: ', command)

        result = np.matmul(self.Jacobian, command)

        print('Result:', result)

        # Send velocity

        msgFloat = Float64()
        msgFloat.data = result[0]
        self.pub_left_wheel.publish(msgFloat)

        msgFloat = Float64()
        msgFloat.data = result[1]
        self.pub_right_wheel.publish(msgFloat)