#! /usr/bin/env python

import rospy
import time
import numpy as np
from config import *
from sensor_msgs.msg import LaserScan

class LaserTopicReader(object):
    def __init__(self, topic_name = '/scan'):
        self._topic_name = topic_name
        self._sub = rospy.Subscriber(self._topic_name, LaserScan, self.topic_callback)
        self._laserdata = LaserScan()
        self._front = 0.0
        self._right = 0.0
        self._left = 0.0
    
    def topic_callback(self, msg):
        self._laserdata = msg
        # rospy.logdebug(self._laserdata)
    
    def get_laserdata(self):
        """
        Returns the newest odom data

        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        float32 angle_min
        float32 angle_max
        float32 angle_increment
        float32 time_increment
        float32 scan_time
        float32 range_min
        float32 range_max
        float32[] ranges
        float32[] intensities
        """
        return self._laserdata
        
    def wall_detector(self):
        self._laserdata
        # rospy.loginfo(self._laserdata)
        self._front = np.min(self._laserdata.ranges[LASER_FRONT-LASER_MARGIN_FRONT:LASER_FRONT+LASER_MARGIN_FRONT])
        self._left = np.min(self._laserdata.ranges[LASER_LEFT-LASER_MARGIN_SIDE:LASER_LEFT+LASER_MARGIN_SIDE])
        self._right = np.min(self._laserdata.ranges[LASER_RIGHT-LASER_MARGIN_SIDE:LASER_RIGHT+LASER_MARGIN_SIDE])
        
        # rospy.loginfo("%.2f - %.2f", self._front, self._right)
        return self.convert_to_dict()
        
    def convert_to_dict(self):
        """
        Converts the fiven message to a dictionary telling in which direction there is a detection
        """
        detect_dict = {}
        # We consider that when there is a big Z axis component there has been a very big front crash
        detection_dict = {"front":self._front,
                          "left":self._left,
                          "right":self._right}
        return detection_dict
        
if __name__ == "__main__":
    rospy.init_node('laser_topic_subscriber', log_level=rospy.INFO)
    laser_reader_object = LaserTopicReader()
    time.sleep(2)
    rate = rospy.Rate(0.5)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        data = laser_reader_object.get_laserdata()
        laser_reader_object.wall_detector()
        rate.sleep()