#! /usr/bin/python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class SendGoalClient:

    def __init__(self):

        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        self.client.wait_for_server()

    def send_goal(self, lx, ly, az=0):
        """
        lx: float
            linear x
        ly: float
            linear y
        az: float
            angular z (yaw) [default: 0 radians]
        """

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = lx
        goal.target_pose.pose.position.y = ly

        orientation = quaternion_from_euler(0, 0, az)
        goal.target_pose.pose.orietnation = orientation

        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        return result

def main():

    try:

        rospy.init_node("send_goal_client_node")

        sgc = SendGoalClient()

    except rospy.ROSInterruptException as e:

        print(e)

if __name__ == "__main__":

    main()