#!/usr/bin/env python
import rospy
from roslib import message
import ros_numpy
import time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Empty


# # fly forward
# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
# # fly backward
# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
# # fly to left
# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 1.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
# # fly to right
# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: -1.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
# # fly up
# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
# # fly down
# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: -1.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
# # counterclockwise rotation
# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 1.0}}'
# # clockwise rotation
# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: -1.0}}'
# # stop
# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'

class ingenuity():

    def __init__(self):
        self.takeoff_pub = rospy.Publisher("ardrone/takeoff", Empty, queue_size=10 )
        self.land_pub = rospy.Publisher("ardrone/land", Empty, queue_size=10 )
        self.move_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.sleep = rospy.sleep(10)
        self.spin = rospy.spin()

    def takeoff(self):
        self.takeoff_pub.publish(Empty())
        self.sleep

    def land(self):
        self.land_pub.publish(Empty())
        self.sleep

    def move(self,lin_x=0,lin_y=0,lin_z=0,ang_x=0,ang_y=0,ang_z=0):
        self.commands = Twist()
        self.commands.linear.x = lin_x 
        self.commands.linear.y = lin_y
        self.commands.linear.z = lin_z
        self.commands.angular.x = ang_x
        self.commands.angular.y = ang_y
        self.commands.angular.z = ang_z
        self.move_pub.publish(self.commands)
        self.sleep

if __name__ == '__main__':

    rospy.init_node('drone', anonymous=True)
    drone = ingenuity()
    drone.takeoff()
    # drone.move(0,0,0,0,0,0)

    # try:
    #     # drone.takeoff()
    #     # #rospy.sleep(5)
    #     # # drone.land()
    #     # while not rospy.is_shutdown():
    #     #     drone.move(0,0,0,0,0,0)
    # except rospy.ROSInterruptException:
    #     pass
