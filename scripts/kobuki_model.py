#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import numpy as np
from rospy.numpy_msg import numpy_msg

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

        # Polling
        
        while not rospy.is_shutdown():
            rospy.loginfo('>> Waiting...')

            # Keyboard commands

            command_char = input()
            rospy.loginfo('>> Command received: ' + command_char)

            if command_char == 'q':
                rospy.loginfo('>> STATUS: Quit.')
                rospy.signal_shutdown('Request shutdown')
            
            elif command_char == 's':
                rospy.loginfo('>> STATUS: Stopped.')
                msf_Float = Float64()
                msf_Float = 0.0
                # Publish
                self.pub_left_wheel.publish(msf_Float)
                self.pub_right_wheel.publish(msf_Float)
            
            elif command_char == 'w':
                rospy.loginfo('>> STATUS: Move forward.')
                msf_Float = Float64()
                msf_Float = 1.0
                # Publish
                self.pub_left_wheel.publish(msf_Float)
                self.pub_right_wheel.publish(msf_Float)

            elif command_char == 'r':
                rospy.loginfo('>> STATUS: Rotate.')
                msf_Float = Float64()
                msf_Float = 1.0
                # Publish
                self.pub_left_wheel.publish(msf_Float)
                self.pub_right_wheel.publish(-msf_Float)


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