#!/usr/bin/env python

#This is a PID controller to determine twist values

import rospy
from state_definitions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

#Linear speed of the robot
LINEAR_SPEED = 0.3
#Angular speed of the robot
ANGULAR_SPEED = 3.1415926/6

#Multipliers used to tune the PID controller
#Proportional constant
P_CONSTANT = 0.7
#Integral constant
I_CONSTANT = 0
#Derivative constant
D_CONSTANT = 0.1  #was 0.1 and outside box worked

#desired distance
FOLLOW_DISTANT= 0.5

#CALLBACKS FOR ANYTHING YOUR PID NEEDS TO SUBSCRIBE TO FROM scan_values_handler

#Init node
rospy.init_node('pid')

#Create publisher for suggested twist objects
pub = rospy.Publisher('pid_twist', Twist, queue_size = 1)

#SUBSCRIBERS FOR THINGS FROM scan_values_handler YOU MIGHT WANT
min_dist = 0
min_angle = 0

def cb_dist(msg):
    global min_dist
    min_dist = msg.data

def cb_angle(msg):
    global min_angle
    min_angle = msg.data

sub_min_dist = rospy.Subscriber('min_dist', Float32, cb_dist)
sub_min_angle  = rospy.Subscriber('min_angle', Float32, cb_angle)



#Twist and rate object
t = Twist()
rate = rospy.Rate(10)




while not rospy.is_shutdown():
    #calculate p component
    p_component =  -(FOLLOW_DISTANT - min_dist)
    #calculate d component
    d_component = -(90 - min_angle)   #diff between angke and min angle
    #calculate i component
    i_component = 0
    #Add them all together, multiplied by their respective tuning values, and multiply everything
    #by the angular velocity
    t.angular.z = ANGULAR_SPEED * (P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component)
    t.linear.x = LINEAR_SPEED
    #Publish the twist to the driver
    print(d_component)
    pub.publish(t)
    rate.sleep() 