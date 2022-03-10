#!/usr/bin/env python

#This processes all of the scan values


import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from state_definitions import *

#Process all the data from the LIDAR
def cb(msg):
    
    #CALCULATE AND PUBLISH ANYTHING ELSE FOR THE PID


    dist_list=[]

    min_angle = None
    min_dist = msg.range_max
    #find min angle and dist. around the robot
    for i, dist in enumerate(msg.ranges):
        if dist > msg.range_min and dist < msg.range_max:
            if min_dist > dist:
                min_angle = i *len(msg.ranges)/360
                min_dist = dist
  


    if min_dist>= 1.5: 
        s = 0
    elif min_angle > 45  and min_angle < 135:
        print(min_angle)
        s = 2
    else:
        s = 1 

    #Determine state
    pub_state.publish(s)
    
    pub_min_angle.publish(min_angle)
    pub_min_dist.publish(min_dist)



#Init node
rospy.init_node('scan_values_handler')

#Subscriber for LIDAR
sub = rospy.Subscriber('scan', LaserScan, cb)

#Publishers
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
#THINK OF WHAT INFO TO PUBLISH TO THE PID

#min angle min dist
pub_min_angle = rospy.Publisher('min_angle', Float32, queue_size = 1)

pub_min_dist = rospy.Publisher('min_dist', Float32, queue_size = 1)

#Rate object
rate = rospy.Rate(10)

#Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 