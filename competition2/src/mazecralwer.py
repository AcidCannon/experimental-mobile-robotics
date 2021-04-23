#!/usr/bin/env python

#outside maze x = 0.32 y = -9.25
# start (0,0,0)
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

def setvelocity(newvelocity):
    velocity_publisher.publish(newvelocity)
    if longsleep == True:
        rospy.sleep(0.45)

        
def laser_handler(msg):
    #msg.ranges
    laserData = msg
    newvelocity = Twist()
    newvelocity.linear.x = 0.3
    newvelocity.angular.z = 0
    longsleep = False
    # wayy too close to right wall turn left
    if laserData.ranges[180] < 0.10 or laserData.ranges[0] < 0.10:
        newvelocity.angular.x = 0
        newvelocity.angular.z = 0.25
        if laserData.ranges[180] > 1.5:
            newvelocity.angular.x = 0.2
        velocity_publisher.publish(newvelocity)
        print("too close to right")
        return 

    if laserData.ranges[360] > 0.7 and laserData.ranges[270] >= 0.4 :
        # break on right wall turn right
        
        if  laserData.ranges[90] > 0.5 or laserData.ranges[0] > 0.5:
            newvelocity.linear.x = 0.15
            newvelocity.angular.z = -0.5
            print("turning right")
            velocity_publisher.publish(newvelocity)
            return 
        # close enough to solid right wall
        if  (laserData.ranges[180] < 0.5 or laserData.ranges[0] < 0.5) :
            newvelocity.linear.x = 0.3
            newvelocity.angular.z = 0
            # too close to wall turn a bit left 
            if laserData.ranges[180] < 0.3 or laserData.ranges[90] <0.3 or laserData.ranges[0] < 0.30:
                newvelocity.angular.z = 0.3
                print("slight left")
                
            print("go streight")
            velocity_publisher.publish(newvelocity)
            return 
        
        # drifting left turn right 
        if  laserData.ranges[180] >= 0.5:
            newvelocity.linear.x = 0.3
            newvelocity.angular.z = -0.15
            print("glid right")
            velocity_publisher.publish(newvelocity)
            return 

    longsleep = True

    # if something is in front and nothing on either side turn left
    if (laserData.ranges[360] <= 0.7 or laserData.ranges[270] <=0.3 ) and laserData.ranges[0] > 0.5 and laserData.ranges[719] > 0.5:
        newvelocity.linear.x = 0
        newvelocity.angular.z = 0.7
        print("wall ahead and nothing beside me turn left")
        velocity_publisher.publish(newvelocity)
        return

    # if something is in front and nothing on my right turn right
    if laserData.ranges[360] <= 0.7 and laserData.ranges[0] > 0.5 and laserData.ranges[180] > 0.5 :
        newvelocity.linear.x = 0
        newvelocity.angular.z = -0.7
        print("wall ahead and nothing right turn right")
        velocity_publisher.publish(newvelocity)
        return 
    
    # if something is in front and something on my right turn left
    if laserData.ranges[360] <= 0.7 and laserData.ranges[0] <= 0.5:
        newvelocity.linear.x = 0
        newvelocity.angular.z = 0.7
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
    
    
    # turtle_subscriber = turtle_topics()

    # service = serviceServer(turtle_subscriber)
    # rospy.wait_for_service('/check_state')
    # service_connection = rospy.ServiceProxy('/check_state', SetBool)
    # service_request_obj = SetBoolRequest()
    
    # action_server = turtle_actions(turtle_subscriber)
    # action_client = actionlib.SimpleActionClient('/turtle_action_server', TestAction)
    # action_client.wait_for_server()
    # goal = TestActionGoal()
    # goal.goal = 1
    
    
    # now = time.time()
    
    # while(True):
        
        # rospy.sleep(0.05)
        # action_client.send_goal(goal, feedback_cb=action_feedback)
       
        # while(action_client.get_state() < 2):
        #     pass
        # service_connection(service_request_obj)
        
        # if(action_server.get_finished()):
        #     service_request_obj.data = True 
        #     service_connection(service_request_obj)
        #     break
        # time.sleep(0.02)

    # print( "Finished in: "+ str(time.time()- now) + "seconds" )
    
    


    rospy.spin()