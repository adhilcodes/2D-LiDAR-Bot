#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SensorCheck:
    def __init__(self):
        self.sub_topic_name = "/scan"
        self.lidar_subscriber = rospy.Subscriber(self.sub_topic_name, LaserScan, self.lidar_cb)

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.vel = Twist()

    def lidar_cb(self, data):
        print("Number of ranges:", len(data.ranges))
        print("Minimum distance to an object:", min(data.ranges))
        print("Maximum distance to an object:", max(data.ranges))

        back = min(data.ranges[0:45] + data.ranges[315:360]) 
        left = min(data.ranges[46:135])
        front = min(data.ranges[136:225]) 
        right = min(data.ranges[226:314])
        

            
        if front < 0.98:
            # while front != float("inf"):
            if left > right:
                rospy.loginfo("Obstacle in front, turning left.")
                self.vel.angular.z = 0.5 
                self.vel.linear.x = 0.1 
                self.pub.publish(self.vel)
            else:
                rospy.loginfo("Obstacle in front, turning right.")
                self.vel.angular.z = -0.5  
                self.vel.linear.x = 0.1
                self.pub.publish(self.vel)
            
        else:
            rospy.loginfo("Path is clear, moving forward.")
            self.vel.linear.x = 0.1

        self.pub.publish(self.vel)


if __name__ == '__main__':
    rospy.init_node('lidar_check') 
    SensorCheck()
    rospy.spin()
