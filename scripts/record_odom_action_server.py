#! /usr/bin/env python
import rospy
import actionlib
import numpy as np
import copy 
from config import *
from robot_control_main.msg import OdomRecordFeedback, OdomRecordResult, OdomRecordAction
from nav_msgs.msg import Odometry
from odom_subscriber import OdomTopicReader
from geometry_msgs.msg import Point, Point32

class RecordOdomClass(object):
    def __init__(self):
        """
        It starts an action Server. 
        """
        # Creates an object that reads from the topic Odom
        self._odom_reader_object = OdomTopicReader()
        self._rate = rospy.Rate(ACTION_RATE)

        # Gets the initial position
        self._init_pose = self._odom_reader_object.get_pose()
        self._init_pose = copy.deepcopy(self._init_pose)
        
        # Initializes the previous position for the distance calculation
        self._prev_pose = self._init_pose

        # Define the current position
        self._pose = Point32()

        # create messages that are used to publish result
        self._result = OdomRecordResult()

        # Define the cumulative distabce covered so far whicn is also the feedack
        self._distance = 0
        self._feedback = OdomRecordFeedback()
   
        # Defines the distance to initial position
        self._distance_init = 0

        # creates the action server
        self._as = actionlib.SimpleActionServer("/record_odom", OdomRecordAction, self.as_callback, False)
        self._as.start()
        rospy.loginfo("Record Odometry Action Server Up")

    def as_callback(self, goal):
        while self._distance < ACTION_MIN_DISTANCE or self._distance_init > ACTION_DISTANCE_INIT_MARGIN:
            self._pose = self._odom_reader_object.get_pose()
            self._distance += self.calculate_distance(self._pose, self._prev_pose)
            self._distance_init = self.calculate_distance(self._pose, self._init_pose)
            self._feedback.current_total = self._distance
            # rospy.loginfo(self._distance_init)
            self._as.publish_feedback(self._feedback)
            self._prev_pose = copy.deepcopy(self._pose)
            self._result.list_of_odoms.append(self._prev_pose)
            self._rate.sleep()
        
        rospy.loginfo("Total distance: " + str(self._distance) + " Distance from init: " + str(self._distance_init))
        self._as.set_succeeded(self._result)

    def calculate_distance(self, p1, p2):
        return np.linalg.norm(np.array([p1.x, p1.y]) - np.array([p2.x, p2.y])) 

if __name__ == '__main__':
    rospy.init_node('record_odom_action_server_node')
    RecordOdomClass()
    rospy.spin()