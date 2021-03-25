#! /usr/bin/env python

##### LASER subscriber parameters  ########
SIMULATION = True
LASER_FRONT = 180 # front reading index
LASER_LEFT = 270 if SIMULATION else 90
LASER_RIGHT = 90 if SIMULATION else 270
LASER_MARGIN_FRONT = 10 # front reading margin in degrees for better accuracy
LASER_MARGIN_SIDE = 5 # side reading margin in degrees

##### Main navigation Parameters   ########
NAV_FRONT_MARGIN = 0.5 # front distance to trigger rotation
NAV_RIGHT_MARGIN_MIN = 0.2 # minimum distance to the right to start moving away
NAV_RIGHT_MARGIN_MAX = 0.3 # maximum distance to the right to start moving towards
NAV_LINEAR_SPEED = 0.1  # fixed linear speed
NAV_ANGULAR_SPEED_LOW = 0.05 # fixed angular speed while navigating
NAV_ANGULAR_SPEED_HIGH = 0.3 # fixed angular speed to avoid obstacle
NAV_RATE = 10 

###### Service Find Wall Parameters ####### 
SRV_ROTATION_MARGIN = 3 
SRV_TARGET_FRONT_ANGLE = LASER_FRONT
SRV_ROTATION_SPEED = 0.1
SRV_TARGET_FRONT_DISTANCE = 0.3
SRV_LINEAR_SPEED = 0.1
SRV_TARGET_RIGHT_ANGLE = LASER_RIGHT
SRV_RATE = 5

###### Action Odometry Parameters  ########
ACTION_RATE = 1
ACTION_DISTANCE_INIT_MARGIN = 0.25 # Stop if robot gets close to initial position by this distance  
ACTION_MIN_DISTANCE = 1 # 1m minimum distance in case robot turns around by error
ACTION_MAX_DISTANCE = 10 # if beyond 10m stop as the stopping condition didn't work