#!/usr/bin/env python3
import rospy
from math import radians 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

MOVE_TO_WALL = 1
FOLLOW_WALL = 2
CORNER = 3 

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.state = MOVE_TO_WALL
        # denotes whether new scan data will be used or ignored
        self.data_needed = True
        self.following_distance= 0.5

    def scan_callback(self, data):
        if not self.data_needed:
            # We get here if we're still processing last data
            return     
        elif self.state == MOVE_TO_WALL:
            self.move_to_wall(data)
        elif self.state == FOLLOW_WALL:
            self.follow_wall(data)
        elif self.state == CORNER:
            self.handle_corner(data)

    def compute_PID(self, error, k):
        return error * k

    def argmin_min(self, arr):
        """
        Returns (argmin, min) given an array
        """
        min_index, min_val= 0, arr[0]
        for (i, val) in enumerate(arr):
            if val < min_val:
                min_index, min_val = i, val 
        return (min_index, min_val)

    def orient_needed(self, nearest_ang=None, desired_ang=270, tolerance=3):
        """
        Returns true if robot's nearest angle is within tolerance of desired_ang
        """
        if not nearest_ang:
            nearest_ang, _ = self.argmin_min(data.ranges)

        return nearest_ang not in range(desired_ang - tolerance, desired_ang + tolerance)

    def orient(self, nearest_degree_ang, desired_degree_ang=270):
        print('\norienting, nearest_ang = ', nearest_degree_ang)
        nearest_ang = radians(nearest_degree_ang)
        desired_ang = radians(desired_degree_ang)
        ang_err = nearest_ang - desired_ang
        self.twist.angular.z = self.compute_PID(error=ang_err, k=0.3)
        self.twist.linear.x = 0        
        self.twist_pub.publish(self.twist)


    def move_to_wall(self, data):
        print('\nmove_to_wall')
        # Process current data before we get more
        self.data_needed = False
        close = 0.5
        nearest_ang, nearest_dist = self.argmin_min(data.ranges)
        print('nearest_ang, nearest_dist', nearest_ang, nearest_dist)
        # If we are already close to a wall and should change states
        # Follow wall will also turn if needed
        if nearest_dist < self.following_distance:
            self.data_needed = True
            # orient such that the angle of closest wall is on the robot's left 
            # If we always turn left, this means the wall we're following is at 
            # 270 degrees anticlockwise from the front of robot
            if self.orient_needed(nearest_ang):
                self.orient(nearest_ang)
            else:    
                self.twist.linear.x = self.twist.angular.z = 0
                self.twist_pub.publish(self.twist)
                self.state = FOLLOW_WALL
            return   

        nearest_ang = radians(nearest_ang) 
        # Use PID on angle adjustments. 
        # (current angle = 0, desired_ang = nearest_ang, so nearest_ang - 0 is err)
        speed = nearest_dist / 5
        self.twist.angular.z = self.compute_PID(error=(nearest_ang - 0), k=0.01)
        self.twist.linear.x = speed 
        self.twist_pub.publish(self.twist) 
        # Need to make sure we have gotten close to wall before change state
        self.data_needed = True
 
    def follow_wall(self, data):
        print('\nin follow_wall')
        self.data_needed = False
        # Follow wall looks at the distance to the wall at angle 225 and 315
        # If we are going straigth along wall, the distances should be ~equal
        # Based on the difference in lengths, we can adjust the angular velocity
        # to straighten out until a corner.
        ang1, ang2 = 225, 315
        length1, length2 = data.ranges[ang1], data.ranges[ang2] 
        print('length1, length2', length1, length2)
        nearest_ang, nearest_dist = self.argmin_min(data.ranges)
        print('nearest_ang, nearest_dist', nearest_ang, nearest_dist)

        close = .1
        if (nearest_ang > 355 or nearest_ang < 5) \
           and (length1 - length2) < close
            self.state = CORNER
            self.twist.linear.x = self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
            self.data_needed = True
            return        
        # To keep straight, we set angular.z to be TURN_magnitude in 
        # the direction we need to adjust in depending on length1 and length2
        # When trying to understand, remember that the angles are anticlockwise from front
        TURN_MAGNITUDE = 0.1
        if length1 < length2: 
            print('length1 > length2, turn right')
            # Turn right, or with negative angular velocity
            self.twist.angular.z = -1 * TURN_MAGNITUDE
        else:
            print('length1 < length2, turn left')
            self.twist.angular.z = TURN_MAGNITUDE 
        self.twist.linear.x = 0.1
        self.twist_pub.publish(self.twist)
        self.data_needed = True

    def handle_corner(self, data):
        print('in handle_corner')
        nearest_ang, _ = self.argmin_min(data.ranges)
        if self.orient_needed(nearest_ang):
            self.orient(nearest_ang)
        else:
            self.twist.linear.x = self.twist.angular.z = 0
            self.twist_pub.publish(self.twist)
            self.state = FOLLOW_WALL
            
 
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    follower = WallFollower()
    follower.run()
