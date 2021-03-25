#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

class OdomTopicReader(object):
    def __init__(self, topic_name = '/odom'):
        self._topic_name = topic_name
        self._check_odom_ready()
        self._sub = rospy.Subscriber(self._topic_name, Odometry, self.topic_callback)
        self._pose = Point()

    def _check_odom_ready(self):
        self._odomdata = None
        while self._odomdata is None and not rospy.is_shutdown():
            try:
                self._odomdata = rospy.wait_for_message(self._topic_name, Odometry, timeout=1.0)
                rospy.logdebug("Current "+self._topic_name+" READY=>" + str(self._odomdata))

            except:
                rospy.logerr("Current "+self._topic_name+" not ready yet, retrying for getting "+self._topic_name+"")
        
        # rospy.loginfo("Initialized Odometry Data="+str(self._odomdata))
        # rospy.loginfo("Initialized Odometry Action")
        return self._odomdata

    def topic_callback(self, msg):
        self._odomdata = msg
        rospy.logdebug(self._odomdata)

    
    def get_odomdata(self):
        """
        Returns the newest odom data
        std_msgs/Header header     
          uint32 seq                                       
          time stamp                                                    
          string frame_id                                               
        string child_frame_id                                           
        geometry_msgs/PoseWithCovariance pose                                                                                                  
          geometry_msgs/Pose pose                                                                                                              
            geometry_msgs/Point position                                                                                                       
              float64 x                                                                                      
              float64 y                                                                                           
              float64 z                                                                                          
            geometry_msgs/Quaternion orientation   
              float64 x                                                                                                                        
              float64 y                                                                                                                        
              float64 z                                                                                                                        
              float64 w                                                                                                                        
          float64[36] covariance                                                                                                               
        geometry_msgs/TwistWithCovariance twist                                                                                                
          geometry_msgs/Twist twist                                                                                                            
            geometry_msgs/Vector3 linear                                                                                                       
              float64 x                                                                                                                        
              float64 y                                                                                                                        
              float64 z                                                                                                                        
            geometry_msgs/Vector3 angular                                                                                                      
              float64 x                                                                                                                        
              float64 y                                                                                                                        
              float64 z                                                                                                                        
          float64[36] covariance                                                                                                               
        
        """
        return self._odomdata

    def get_pose(self):
        """
        Procecess the Odom data and return x,y,theta Point of robot 
        """
        self._pose.x = self._odomdata.pose.pose.position.x
        self._pose.y = self._odomdata.pose.pose.position.y
        orientation_q = self._odomdata.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self._pose.z ) = euler_from_quaternion(orientation_list)
        #self._pose.z = self._odomdata.pose.pose.orientation.z
        return self._pose
    
if __name__ == "__main__":
    rospy.init_node('odom_topic_subscriber', log_level=rospy.INFO)
    odom_reader_object = OdomTopicReader()
    # rospy.loginfo(odom_reader_object.get_pose())
    rate = rospy.Rate(ACTION_RATE)
    
    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print("shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    
    while not ctrl_c:
        data = odom_reader_object.get_pose()
        rospy.loginfo(data)
        rate.sleep()