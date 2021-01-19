#!/usr/bin/env python3
from math import pi
from time import sleep
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class DriveSquare(object):

    def __init__(self):
        rospy.init_node('drive_square')
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Give time for publisher to register
        rospy.sleep(1)
        lin = Vector3(x=0, y=0, z=0)
        ang = Vector3(x=0, y=0, z=0)
        self.twist = Twist(linear=lin, angular=ang)
        self.twist_pub.publish(self.twist)

    def run(self):
        move_speed = 0.2
        # Time to spend drawing the lines of the square
        move_time = 3
        # Time to spend turning; total turn will be 90 degrees regardless
        # of valiue
        turn_time = 2
        turn_speed = (pi / 2) / turn_time # 90 degrees in turn_time secs
        while True:
            self.twist.linear.x = move_speed
            # Next line does nothing on first iteration
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
            rospy.sleep(move_time)
            self.twist.linear.x = 0 
            self.twist.angular.z = turn_speed 
            self.twist_pub.publish(self.twist)
            rospy.sleep(turn_time)

if __name__ == '__main__':
    node = DriveSquare()
    node.run()

    
