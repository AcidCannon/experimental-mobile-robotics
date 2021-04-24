#!/usr/bin/env python

import rospy, time, signal, sys, actionlib
import math
import test

from std_srvs.srv import SetBool, SetBoolRequest
from actionlib.msg import TestAction, TestActionFeedback, TestActionGoal

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def signal_handler(sig, frame):
    sys.exit(0)


position = Odometry()
laserData = LaserScan()
laserData.ranges = [0]*720
longsleep = False

laser_subscriber = None
velocity_publisher = None
scale = 2

def setvelocity(newvelocity):
    velocity_publisher.publish(newvelocity)
    if longsleep == True:
        rospy.sleep(0.45)

        
def laser_handler(msg):
    #msg.ranges
    laserData = msg
    newvelocity = Twist()
    newvelocity.linear.x = 0.3*scale
    newvelocity.angular.z = 0
    longsleep = False
    # wayy too close to right wall turn left
    if laserData.ranges[180] < 0.10 or laserData.ranges[0] < 0.10:
        newvelocity.angular.x = 0
        newvelocity.angular.z = 0.25*scale
        if laserData.ranges[180] > 1.5:
            newvelocity.angular.x = 0.2*scale
        velocity_publisher.publish(newvelocity)
        print("too close to right")
        return 

    if laserData.ranges[360] > 0.7 and laserData.ranges[270] >= 0.4 :
        # break on right wall turn right
        
        if  laserData.ranges[90] > 0.5 or laserData.ranges[0] > 0.5:
            newvelocity.linear.x = 0.15*scale
            newvelocity.angular.z = -0.5*scale
            print("turning right")
            velocity_publisher.publish(newvelocity)
            return 
        # close enough to solid right wall
        if  (laserData.ranges[180] < 0.5 or laserData.ranges[0] < 0.5) :
            newvelocity.linear.x = 0.3*scale
            newvelocity.angular.z = 0
            # too close to wall turn a bit left 
            if laserData.ranges[180] < 0.3 or laserData.ranges[90] <0.3 or laserData.ranges[0] < 0.30:
                newvelocity.angular.z = 0.3*scale
                print("slight left")
                
            print("go streight")
            velocity_publisher.publish(newvelocity)
            return 
        
        # drifting left turn right 
        if  laserData.ranges[180] >= 0.5:
            newvelocity.linear.x = 0.3*scale
            newvelocity.angular.z = -0.15*scale
            print("glid right")
            velocity_publisher.publish(newvelocity)
            return 

    longsleep = True

    # if something is in front and nothing on either side turn left
    if (laserData.ranges[360] <= 0.7 or laserData.ranges[270] <=0.3 ) and laserData.ranges[0] > 0.5 and laserData.ranges[719] > 0.5:
        newvelocity.linear.x = 0
        newvelocity.angular.z = 0.7*scale
        print("wall ahead and nothing beside me turn left")
        velocity_publisher.publish(newvelocity)
        return

    # if something is in front and nothing on my right turn right
    if laserData.ranges[360] <= 0.7 and laserData.ranges[0] > 0.5 and laserData.ranges[180] > 0.5 :
        newvelocity.linear.x = 0
        newvelocity.angular.z = -0.7*scale
        print("wall ahead and nothing right turn right")
        velocity_publisher.publish(newvelocity)
        return 
    
    # if something is in front and something on my right turn left
    if laserData.ranges[360] <= 0.7 and laserData.ranges[0] <= 0.5:
        newvelocity.linear.x = 0
        newvelocity.angular.z = 0.7*scale
        print("wall ahead and right blocked")
        velocity_publisher.publish(newvelocity)
        return

    print("6")
    velocity_publisher.publish(newvelocity)
    return
    

if __name__ == "__main__":

    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('turtle')
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1 )
    laser_subscriber = rospy.Subscriber("/kobuki/laser/scan", LaserScan, laser_handler)
    
    rospy.spin()