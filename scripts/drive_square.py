#!/usr/bin/env python3
from time import sleep
from math import pi
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class DriveSquare(object):

    def __init__(self):
        rospy.init_node('drive_square')
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Give time for publisher to register
        sleep(1)
        lin = Vector3(x=0, y=0, z=0)
        ang = Vector3(x=0, y=0, z=0)
        self.twist = Twist(linear=lin, angular=ang)
        self.twist_pub.publish(self.twist)

    def run(self):
        move_speed = 0.2
        move_time = 3
        turn_time = 2
        turn_speed = (pi / 2) / turn_time # 90 degrees in turn_time secs
        for _ in range(16):
            self.twist.linear.x = move_speed
            # Next line does nothing on first iteration
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
            sleep(move_time)
            self.twist.linear.x = 0 
            self.twist.angular.z = turn_speed 
            self.twist_pub.publish(self.twist)
            sleep(turn_time)
            self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
            sleep(2)

if __name__ == '__main__':
    node = DriveSquare()
    node.run()

    
