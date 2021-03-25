#! /usr/bin/env python

import rospy
import time
import actionlib
from config import *
from cmd_vel_publisher import CmdVelPub
from laser_subscriber import LaserTopicReader
from odom_subscriber import OdomTopicReader
from robot_control_main.srv import FindWall, FindWallResponse, FindWallRequest
from robot_control_main.msg import OdomRecordFeedback, OdomRecordResult, OdomRecordAction, OdomRecordGoal
from std_srvs.srv import Empty, EmptyRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Point32
from post_processing import *

class RobotControl(object):
    def __init__(self):
        rospy.loginfo("Initializing Robot Control")
        self._rate = rospy.Rate(NAV_RATE)
        self.init_findwall_service_client()
        self.init_rec_odom_action_client()
        self.init_robot_navigation()

    def init_findwall_service_client(self):
        service_name = '/find_wall'
        rospy.loginfo('Waiting for Service Server')
        rospy.wait_for_service(service_name)
        rospy.loginfo('Service Server Found...')
        self._findwall_service = rospy.ServiceProxy(service_name, FindWall)
        self._request_object = FindWallRequest()

    def make_findwall_request(self):
        rospy.loginfo('Service request done')
        result = self._findwall_service(self._request_object)
        return result

    def init_rec_odom_action_client(self):
        self._rec_odom_action_client = actionlib.SimpleActionClient('/record_odom',OdomRecordAction)
        # waits until the action server is up and running
        rospy.loginfo('Waiting for action Server')
        self._rec_odom_action_client.wait_for_server()
        rospy.loginfo('Action Server Found...')
        self._rec_odom_action_goal = OdomRecordGoal()

    def send_goal_to_rec_odom_action_server(self):
        self._rec_odom_action_client.send_goal(self._rec_odom_action_goal, feedback_cb=self.rec_odom_feedback_callback)

    def rec_odom_feedback_callback(self,feedback):
        rospy.loginfo("Rec Odom Feedback feedback ==>"+str(feedback))

    def rec_odom_finished(self):
        has_finished = ( self._rec_odom_action_client.get_state() >= 2 )
        return has_finished

    def get_result_rec_odom(self):
        return self._rec_odom_action_client.get_result()
    
    def init_robot_navigation(self):
        self._cmdvelpub_object = CmdVelPub()
        self._lasersub_object = LaserTopicReader()
        time.sleep(1) # time to initialize

    def navigate_robot(self):
        while(not self.rec_odom_finished()):
            laser_reading = self._lasersub_object.wall_detector()
            if laser_reading['front'] < NAV_FRONT_MARGIN:
                self.move_robot('forwardsleft', linearspeed=NAV_LINEAR_SPEED, angularspeed=NAV_ANGULAR_SPEED_HIGH)
            elif laser_reading['right'] >= NAV_RIGHT_MARGIN_MAX:
                self.move_robot('forwardsright', linearspeed=NAV_LINEAR_SPEED, angularspeed=NAV_ANGULAR_SPEED_LOW)
            elif laser_reading['right'] < NAV_RIGHT_MARGIN_MIN:
                self.move_robot('forwardsleft', linearspeed=NAV_LINEAR_SPEED, angularspeed=NAV_ANGULAR_SPEED_LOW)
            else:
                self.move_robot('forwards', linearspeed=NAV_LINEAR_SPEED)
            self._rate.sleep()

        robot_control_object.move_robot('stop')
        self._rate.sleep()

    def move_robot(self, direction, linearspeed=0,angularspeed=0):
        self._cmdvelpub_object.move_robot(direction,linearspeed=linearspeed, angularspeed=angularspeed)


rospy.init_node("robot_control_main_node", log_level=rospy.INFO)
robot_control_object = RobotControl()
# rate = rospy.Rate(NAV_RATE)

# Start the service
result = robot_control_object.make_findwall_request()

# When service succeed start the navigation and odometry
if result.wallfound:
    robot_control_object.send_goal_to_rec_odom_action_server()
    robot_control_object.navigate_robot()

odom_result = robot_control_object.get_result_rec_odom()
odom_result_array = odom_result.list_of_odoms

rospy.loginfo("Mission Accomplished")
#print(odom_result_array)
draw_points(odom_result_array)