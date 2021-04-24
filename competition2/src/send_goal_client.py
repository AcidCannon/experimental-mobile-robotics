#! /usr/bin/python

import rospy
import actionlib
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
from tf.transformations import quaternion_from_euler
import rosparam
from pathlib import Path
from move import Move

class SendGoalClient:

    def __init__(self):

        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.locations = rosparam.load_file(str(Path.home()) + "/catkin_ws/src/competition2/yaml/locations.yaml")[0][0]
        self.numbered_locations = rosparam.load_file(str(Path.home()) + "/catkin_ws/src/competition2/yaml/numbered_locations.yaml")[0][0]

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.twist = Twist()

        self.amcl_pose = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.covariance = {}

        # wait for covariance subscriber to receive data
        while len(self.covariance) != 3:
            pass

        self.initial_pose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

        self.move = Move()

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
        while (self.covariance["x"] + self.covariance["y"]) > 0.005:
            print("covaraince:", (self.covariance["x"] + self.covariance["y"]))
            self.rotate("right", 180, z=2.0)
            iteration += 1

            if iteration > 4:
                random = np.random.choice(["forward", "backward", "right", "left"])
                self.move.move(random, distance=1.0)
                iteration = 0


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

    def traverse(self, location):
        """
        Send goal position to move_base action server.
        Traverse the resultant path.

        location: int
            location number
        """

        if location == 0:
            print("\nmoving to lobby")
        else:
            print("\nmoving to room {}".format(location))
        location = self.numbered_locations[location]

        # get location data
        lx = self.locations[location]["lx"]
        ly = self.locations[location]["ly"]
        az = 0.0
        if "az" in self.locations[location]:
            az = self.locations[location]["az"]

        # send location data to service
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

        print("room reached.\n")

    def teleport(self, location):

        if location == 0:
            print("\nteleporting to lobby")
        else:
            print("\nteleporting to room {}".format(location))
        location = self.numbered_locations[location]

        # get location data
        lx = self.locations[location]["lx"]
        ly = self.locations[location]["ly"]
        az = 0.0
        if "az" in self.locations[location]:
            az = self.locations[location]["az"]

        pose = PoseWithCovarianceStamped()

        pose.header.frame_id = "map"

        pose.pose.pose.position.x = lx
        pose.pose.pose.position.y = ly

        orientation = quaternion_from_euler(0, 0, az)
        pose.pose.pose.orientation.x = orientation[0]
        pose.pose.pose.orientation.y = orientation[1]
        pose.pose.pose.orientation.z = orientation[2]
        pose.pose.pose.orientation.w = orientation[3]

        pose.pose.covariance[0] = 4.0
        pose.pose.covariance[7] = 4.0
        pose.pose.covariance[-1] = 4.0

        self.initial_pose.publish(pose)

    def teleport_old(self, location):
        """
        Teleport robot's position.

        location: string
            location name
        """

        if location == 0:
            print("\teleporting to lobby")
        else:
            print("\teleporting to room {}".format(location))
        location = self.numbered_locations[location]

        # get location data
        lx = self.locations[location]["lx"]
        ly = self.locations[location]["ly"]
        az = 0.0
        if "az" in self.locations[location]:
            az = self.locations[location]["az"]

        # send location data to service
        rospy.wait_for_service("/gazebo/set_model_state")

        set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        model_state = SetModelState()

        model_state.model_name = "mobile_base"

        pose = Pose()
        pose.position.x = lx
        pose.position.y = ly

        orientation = quaternion_from_euler(0, 0, az)
        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]

        model_state.pose = pose

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        model_state.twist = twist

        model_state.reference_frame = "map"

        set_model_state(model_state)

        print("reached {}\n".format(location))
