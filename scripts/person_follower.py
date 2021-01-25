#!/usr/bin/env python3
import rospy
from math import radians 

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class PersonFollower:
        
    def __init__(self):
        rospy.init_node('person_follower')
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        linear = Vector3(x=0.1, y=0, z=0)
        angular = Vector3()
        self.twist = Twist(linear=linear, angular=angular)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def scan_callback(self, data):
        nearest_ang, nearest_dist = 0, data.ranges[0]
        for (degree, dist) in enumerate(data.ranges):
            if dist < nearest_dist:
                nearest_ang, nearest_dist = degree, dist
        if nearest_dist == float('inf'):
             self.twist.linear.x = self.twist.angular.z = 0
             self.twist_pub.publish(self.twist)
             return
        tolerance = 10 
        desired_angle = 0
        k = 0.01
        # PID. Desired angle is nearest_ang, current_angle is 0
        # so differenc is just nearest_ang. 
        if nearest_ang > 360 - tolerance or nearest_ang < tolerance:
            self.twist.angular.z = 0
        else:
            self.twist.angular.z = k * nearest_ang
        close = 0.5
        if nearest_dist > close:
            self.twist.linear.x = 0.3
        else:
            self.twist.linear.x = 0
        self.twist_pub.publish(self.twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
        follower = PersonFollower()
        follower.run()
