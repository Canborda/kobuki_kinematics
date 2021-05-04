#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist

from pynput.keyboard import Key, Listener

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


class keyop_publisher:

    def __init__(self):

        # Setup keyboard listener
        self.listener = Listener(on_press=pressed, on_release=released)
        self.listener.start()

        # Define publisher
        self.pub_velocity = rospy.Publisher("/velocity_topic", Twist, queue_size=10)

        # Polling
        self.msg = Twist()
        self.rate = rospy.Rate(10) # publish messages at 10Hz

        # LOOP
        
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
            
            # Publish and wait
            self.pub_velocity.publish(self.msg)
            self.rate.sleep()

            #rospy.loginfo("Key pressed = " + str(current_key))
            print('Key pressed = ', current_key)

if __name__ == '__main__':
    rospy.init_node("keyop_node", anonymous=True)
    rospy.loginfo('>> STATUS: Initialize \"keyop\" node')
    keyop_publisher()
    rospy.spin()