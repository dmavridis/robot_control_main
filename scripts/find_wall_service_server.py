#! /usr/bin/env python

import rospy
import numpy as np
import time
from config import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from robot_control_main.srv import FindWall, FindWallResponse
from cmd_vel_publisher import CmdVelPub
from laser_subscriber import LaserTopicReader


class FindWallService(object):
    def __init__(self):
        self.rotated = False
        self.positioned = False
        self.complete = False # Service is complete
        self._turning_direction = "right"
        self._turning_selected = False
        self._rate = rospy.Rate(SRV_RATE)
        self._cmdvelpub_object = CmdVelPub()
        self._lasersub_object = LaserTopicReader()
        self.srv = rospy.Service('/find_wall', FindWall, self.srv_cb)
        rospy.loginfo("Find Wall Service Server Up")

    # Service Callback
    def srv_cb(self,request):
        self.complete = False # Service can be called again
        # rospy.loginfo("Complete: " + str(self.complete))
        response = False
        rospy.loginfo("Find Wall Service started")
        # Subsriber definition
        response = FindWallResponse()
        
        while not self.complete:
            self.srv_processing()
            self._rate.sleep()

        response.wallfound = True    
        return response

    def srv_processing(self):
        ## message of type LaserScan
        msg = self._lasersub_object.get_laserdata()
        current_position = np.argmin(msg.ranges)

        if not self.rotated and not self.positioned: ## Get the correct rotation
            rospy.loginfo("Rotating #1: %s", current_position)
            target_position = SRV_TARGET_FRONT_ANGLE # front should face the wall
            margin_rotation = SRV_ROTATION_MARGIN
            if not self._turning_selected: # Avoids strange behaviour
                self._turning_direction = 'right' if (target_position - current_position >= 0) else 'left'
                self._turning_selected = True
            if np.abs(target_position - current_position) > margin_rotation:
                self.move_robot(self._turning_direction, angularspeed=SRV_ROTATION_SPEED)
            else:
                self.move_robot('stop')
                self.rotated = True
                rospy.loginfo("Rotating #1 Completed")

        if self.rotated and not self.positioned: # get close to the wall
            target_position = SRV_TARGET_FRONT_ANGLE 
            distance_front = msg.ranges[target_position]
            margin_distance = SRV_TARGET_FRONT_DISTANCE
            rospy.loginfo("Positioning %s", distance_front)
            if distance_front > margin_distance:
                self.move_robot('forwards', linearspeed=SRV_LINEAR_SPEED)
            else:
                self.move_robot('stop')
                self.positioned = True
                self.rotated = False
                rospy.loginfo("Positioning Completed")

        if not self.rotated and self.positioned:
            rospy.loginfo("Rotating #2: %s", current_position)
            target_position = SRV_TARGET_RIGHT_ANGLE # 90 # front should face the wall
            margin_rotation = SRV_ROTATION_MARGIN
            if np.abs(target_position - current_position) > margin_rotation:
                self.move_robot('left', angularspeed=SRV_ROTATION_SPEED)
            else:
                self.move_robot('stop')
                self.rotated = True
                rospy.loginfo("Rotating #2 Completed")

        if self.rotated and self.positioned:
            # rospy.loginfo("Service Completed")
            self.move_robot('stop')
            time.sleep(1)            
            # self.sub.unregister()
            self.complete = True

    def move_robot(self, direction, linearspeed=0,angularspeed=0):
        self._cmdvelpub_object.move_robot(direction,linearspeed=linearspeed, angularspeed=angularspeed)

if __name__ == "__main__":
    rospy.init_node('find_wall_node')
    find_wall_service_object = FindWallService()
    rospy.spin() # maintain the service open