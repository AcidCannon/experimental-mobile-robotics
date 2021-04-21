#! /usr/bin/python

import rospy
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from tf.transformations import quaternion_from_euler
import rosparam

class SendGoalClient:

    def __init__(self):

        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.locations = rosparam.load_file("/home/user/catkin_ws/src/competition2/yaml/locations.yaml")[0][0]

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.twist = Twist()

        self.amcl_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.covariance = {}

        # wait for covariance subscriber to receive data
        while len(self.covariance) != 3:
            pass
        
        self.localize()

    def amcl_pose_callback(self, data):

        self.covariance["x"] = data.pose.covariance[0]
        self.covariance["y"] = data.pose.covariance[7]
        self.covariance["z"] = data.pose.covariance[-1]

    def localize(self):
        """
        Localize the robot.
        """

        print("\nplease wait...attempting to localize")

        iteration = 0
        while (self.covariance["x"] + self.covariance["y"]) > 0.0001 and iteration < 8:
            self.rotate("right", 90, z=2.0)
            iteration += 1

        self.publish()

        print("localization attempt complete\n")

    def publish(self, x=0.0, z=0.0):
        """
        Publish to /cmd_vel topic.

        @param x: float
            linear velocity
        @param z: float
            angular velocity
        """

        self.twist.linear.x = x
        self.twist.angular.z = z

        self.cmd_vel.publish(self.twist)

    def rotate(self, direction, angle, z=1.0):
        """
        Rotate the robot without moving it along the x or y axes.

        @param direction: string
            direction to rotate
        @param angle: float
            angle of rotation
        @param z: float
            angular velocity

        Notes:
        left: +w
        right: -w
        """
        
        v = 0

        if direction == "left":
            w = abs(z)
        elif direction == "right":
            w = -abs(z)

        self.publish(v, w)
        t = np.radians(angle) / z
        rospy.sleep(t)

    def send_goal(self, lx=0.0, ly=0.0, az=0.0):
        """
        Send goal position to move_base action server.

        lx: float
            linear x
        ly: float
            linear y
        az: float
            angular z (yaw) [default: 0 radians]
        """

        print("\nsending goal: lx={}, ly={}, az={}".format(lx, ly, az))

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = lx
        goal.target_pose.pose.position.y = ly

        orientation = quaternion_from_euler(0, 0, az)
        goal.target_pose.pose.orientation.x = orientation[0]
        goal.target_pose.pose.orientation.y = orientation[1]
        goal.target_pose.pose.orientation.z = orientation[2]
        goal.target_pose.pose.orientation.w = orientation[3]

        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        print("reached goal\n")

        return result

def main():

    try:

        rospy.init_node("send_goal_client_node")

        sgc = SendGoalClient()

        while True:

            input_ = input("What room would you like to navigate to?\nSelect any lowercase letter 'a' through 'q' to navigate to a room.\nType 'exit' to quit.\n")

            if input_ == "exit":
                break
            elif input_ in "abcdefghijklmnopq" and len(input_) == 1:
                coordinates = sgc.locations[input_]
                lx = coordinates["lx"]
                ly = coordinates["ly"]
                az = coordinates["az"]
                sgc.send_goal(lx, ly, az)
            else:
                print("You entry is invalid.  Please try again.\n")

    except rospy.ROSInterruptException as e:

        print(e)

if __name__ == "__main__":

    main()