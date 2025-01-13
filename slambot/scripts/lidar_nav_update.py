#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time

class SensorCheck:
    def _init_(self):
        self.sub_topic_name = "/scan"
        self.lidar_subscriber = rospy.Subscriber(self.sub_topic_name, LaserScan, self.lidar_cb)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.vel = Twist()

        # FSM States
        self.state = "MOVING_FORWARD"
        self.turn_start_time = None

        # Parameters
        self.threshold = 1      # Threshold for obstacle detection
        self.clearance_threshold = 1.2  # Clearance to resume forward motion
        self.turn_duration = 2.0    # Turning duration in seconds
        self.forward_speed = 0.15   # Forward linear speed
        self.turn_speed = .9    # Angular turning speed

    def lidar_cb(self, data):
        # Filter out invalid data points (inf or NaN) from ranges
        ranges = np.array(data.ranges)
        ranges = np.where(np.isinf(ranges) | np.isnan(ranges), data.range_max, ranges)

        # Get the minimum distance for each direction
        left=min( data.ranges[0:90]+data.ranges[629:720])*10
        back =min(data.ranges[92:270])*10
        right=min(data.ranges[271:450])*10
        front=min(data.ranges[451:628])*10
        rospy.loginfo(f"Distances -> Front: {front:.2f}, Left: {left:.2f}, Right: {right:.2f}")

        if self.state == "MOVING_FORWARD":
            if front < self.threshold:
                rospy.loginfo("Obstacle detected! Entering turning mode.")
                self.state = "TURNING_LEFT" if left > right else "TURNING_RIGHT"
                self.turn_start_time = time.time()  # Record turn start time
            else:
                self.vel.linear.x = self.forward_speed
                self.vel.angular.z = 0.0

        elif self.state == "TURNING_LEFT":
            if time.time() - self.turn_start_time < self.turn_duration:
                rospy.loginfo("Turning left to avoid obstacle.")
                self.vel.linear.x = 0.05  # Slow forward movement while turning
                self.vel.angular.z = self.turn_speed
            else:
                rospy.loginfo("Turning left complete. Resuming forward motion.")
                self.state = "MOVING_FORWARD"

        elif self.state == "TURNING_RIGHT":
            if time.time() - self.turn_start_time < self.turn_duration:
                rospy.loginfo("Turning right to avoid obstacle.")
                self.vel.linear.x = 0.05  # Slow forward movement while turning
                self.vel.angular.z = -self.turn_speed
            else:
                rospy.loginfo("Turning right complete. Resuming forward motion.")
                self.state = "MOVING_FORWARD"

        # Publish the velocity command
        self.pub.publish(self.vel)

if _name_ == '_main_':
    rospy.init_node('lidar_check') 
    SensorCheck()
    rospy.spin()
