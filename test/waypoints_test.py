#! /usr/bin/env python3

import rospy
import rosunit
import rostest
import unittest
import actionlib
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import tf.transformations as tf
from tortoisebot_waypoints_interface.msg import WaypointActionAction, WaypointActionGoal
import math
import time

PKG = 'tortoisebot_waypoints'
NAME = 'waypoint_test'

########################################################

class TestWaypointActionServer(unittest.TestCase):

    def setUp(self):
        
        #establish client connection
        self.client = actionlib.SimpleActionClient("tortoisebot_as",WaypointActionAction)
        self.client.wait_for_server()
        
        self.current_pos = Point()
        self.des_pos = WaypointActionGoal()
        self.current_yaw = 0.0
        self.des_yaw = 0.0

        self.dist_precision = 0.09
        self.yaw_precision = 0.5

        self.odom_listener = rospy.Subscriber("/odom", Odometry, self.odom_callback)
    def odom_callback(self, msg):

        #convert quaternion to radian
        orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        roll, pitch, yaw = tf.euler_from_quaternion(orientation)
        self.current_pos = msg.pose.pose.position
        self.current_yaw = yaw

    def test_end_robot_pose(self):
        #send a goal and check the result
        self.client.cancel_goal()
        self.des_pos.position.x = 0.2
        self.des_pos.position.y = 0.2
        
        self.client.send_goal(self.des_pos)

        #wait for the result.
        self.client.wait_for_result(rospy.Duration(60))

        if self.client.get_result():
            err_pos = math.sqrt(pow(self.current_pos.x - self.des_pos.position.x,2)+pow(self.current_pos.y - self.des_pos.position.y,2))
            self.des_yaw = math.atan2(self.des_pos.position.y - self.current_pos.y, self.des_pos.position.x - self.current_pos.x)
            err_yaw = self.des_yaw - self.current_yaw

            #compare err_pos with dist_precision and err_yaw with yaw_precision
            self.assertTrue(err_pos <= self.dist_precision, "check distance is within distance tolerance")
            self.assertTrue(abs(err_yaw) <= self.yaw_precision, "check yaw is within yaw tolerance")
            self.assertGreater(abs(self.current_yaw),0.0)
        else:
            #print error message
            print("the waypoints action server is unsuccessful.")
            self.assertTrue(self.client.get_result())
########################################################
if __name__ == '__main__':
    rospy.init_node('my_test_node')
    rostest.rosrun(PKG, NAME, TestWaypointActionServer)