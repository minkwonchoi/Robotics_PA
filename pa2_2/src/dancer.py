#!/usr/bin/env python



#roslaunch turtlebot3_bringup turtlebot3_robot.launch 



# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
from sensor_msgs.msg import LaserScan
from datetime import datetime


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        f
   l    h    r
        b


CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(movement, target_linear_vel, target_angular_vel):
    return "Current Motion: %s \t linear vel %s\t angular vel %s " % (movement, target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel


def scan_cb(message):
    if message.ranges[0]<0.2 and message.ranges[0]>0.01:
        
        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0
        print("                                                                  ", end="\r")
        print(vels("WALL", target_linear_vel, target_angular_vel))
        twist = Twist()
        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
        pub.publish(twist)
        rospy.signal_shutdown(' ')






if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)
    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    



    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'f' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print("                                                                  ", end="\r")
                print(vels('Forward', target_linear_vel,target_angular_vel), end="\r")
            elif key == 'b' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print("                                                                  ", end="\r")
                print(vels('Backward', target_linear_vel,target_angular_vel), end="\r")
            elif key == 'l' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print("                                                                  ", end="\r")
                print(vels('Rotate Left', target_linear_vel,target_angular_vel), end="\r")
            elif key == 'r' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print("                                                                  ", end="\r")
                print(vels('Rotate Right', target_linear_vel,target_angular_vel), end="\r")


            elif key == 's' : #spiraling motion (like a curl or a spring) 
                #make siral with constant velocity and increasing angular velocity
                currtime = rospy.Time.now().to_sec()
               
                #currtime = rospy.Time.now().to_sec()
                   
                while(True):
                    #for i in range(100):
                    target_linear_vel =  (rospy.Time.now().to_sec()-currtime)/50
                    #target_angular_vel = 
                    status = status + 1
                    twist = Twist()
                    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.2
                    pub.publish(twist)


                
                print("                                                                  ", end="\r")
                print(vels('Making Spiral', target_linear_vel,target_angular_vel), end="\r")

                

            elif key == 'z' : #zigzag motion
                for i in range(2):
                    #Turn left
                    currtime = rospy.Time.now().to_sec()
                    while (rospy.Time.now().to_sec()-currtime  < 0.2 ):
                       
                        target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE) 
                        status = status + 1

                        twist = Twist()
                        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                        pub.publish(twist)
                    
                    #Stop
                    currtime = rospy.Time.now().to_sec()
                    while (rospy.Time.now().to_sec()-currtime  < 0.3 ):
                        target_linear_vel   = 0.0
                        target_angular_vel  = 0.0
                        control_linear_vel  = 0.0
                        control_angular_vel = 0.0


                        twist = Twist()
                        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                        pub.publish(twist)

                    #Go forward
                    currtime = rospy.Time.now().to_sec()
                    while (rospy.Time.now().to_sec()-currtime  < 1 ):
                        
                        target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)


                        twist = Twist()
                        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                        pub.publish(twist)
                        
                    #Stop
                    currtime = rospy.Time.now().to_sec()
                    while (rospy.Time.now().to_sec()-currtime  < 0.3 ):
                        target_linear_vel   = 0.0
                        target_angular_vel  = 0.0
                        control_linear_vel  = 0.0
                        control_angular_vel = 0.0


                        twist = Twist()
                        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                        pub.publish(twist)


                    #Turn right
                    currtime = rospy.Time.now().to_sec()
                    while (rospy.Time.now().to_sec()-currtime  < 0.2 ):
                       
                        target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE) 
                        status = status + 1

                        twist = Twist()
                        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                        pub.publish(twist)
                    
                    #Stop
                    currtime = rospy.Time.now().to_sec()
                    while (rospy.Time.now().to_sec()-currtime  < 0.3 ):
                        target_linear_vel   = 0.0
                        target_angular_vel  = 0.0
                        control_linear_vel  = 0.0
                        control_angular_vel = 0.0


                        twist = Twist()
                        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                        pub.publish(twist)

                    #Go forward
                    currtime = rospy.Time.now().to_sec()
                    while (rospy.Time.now().to_sec()-currtime  < 1 ):
                        
                        target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)


                        twist = Twist()
                        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                        pub.publish(twist)
                        
                    #Stop
                    currtime = rospy.Time.now().to_sec()
                    while (rospy.Time.now().to_sec()-currtime  < 0.3 ):
                        target_linear_vel   = 0.0
                        target_angular_vel  = 0.0
                        control_linear_vel  = 0.0
                        control_angular_vel = 0.0


                        twist = Twist()
                        control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                        twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                        control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                        pub.publish(twist)

                print("                                                                  ", end="\r")
                print(vels('Going Zig-Zag', target_linear_vel,target_angular_vel), end="\r")


            elif key == 'd' : 
                #donut movement that F1 players do after a race
                currtime = rospy.Time.now().to_sec()
                counter = 1
                while (rospy.Time.now().to_sec()-currtime  < 1 ):
                    target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                    target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE * (counter%5))
                    status = status + 1
                    print("                                                                  ", end="\r")
                    print(vels('Do a Donut!', target_linear_vel,target_angular_vel), end="\r")
                    

                    twist = Twist()
                    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                    pub.publish(twist)
                    counter+=1



            elif key == ' ' or key == 'h' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print("                                                                  ", end="\r")
                print(vels('Stop', target_linear_vel, target_angular_vel), end="\r")

            


            else:
                if (key == '\x03'):
                    break

            twist = Twist()
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
