#! /usr/bin/python

import rospy
import math
from geometry_msgs.msg import Twist
import numpy as np

class Move:

    def __init__(self):

        self.cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.twist = Twist()

    def move(self, direction, distance=0.2, angle=90.0, radius=1.0, v=1.0):
        """
        Open loop control.

        @param direction: string
            direction to move
        @param distance: float
            if moving forward or backward, distance to move
        @param angle: float
            if moving right or left, angle to rotate
        @param radius: float
            if moving right or left, radius of the movement path
        @param v: float
            linear velocity
        """

        def backward(d):
            radius = float("inf")
            s = -d
            return radius, s

        def forward(d):
            radius = float("inf")
            s = d
            return radius, s

        def left(angle, radius):
            s = abs(radius) * abs(math.radians(angle))
            return abs(radius), s
            
        def right(angle, radius):
            s = abs(radius) * abs(math.radians(angle))
            return -abs(radius), s

        if direction == "forward":
            R, s = forward(distance)
        elif direction == "backward":
            R, s = backward(distance)
            v = -abs(v)
        elif direction == "left":
            R, s = left(angle, radius)
        elif direction == "right":
            R, s = right(angle, radius)

        w = v / R
        self.publish(v, w)
        t = s / v
        rospy.sleep(t)

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

    def stop(self):
        """
        Stop movement.
        """

        self.publish(0.0, 0.0)

def main():

    rospy.init_node("move_node")

    m = Move()

    while True:

        selection = input("Which movement do you want to make? \n f: forward b: backward \n r: right l: left \n in-place rotation: rr: rotate right ll: rotate left \n s: stop e: exit \n Enter the letter and hit enter: ")

        if selection == "f":
            print("\nmoving forward\n")
            m.move("forward")
        elif selection == "b":
            print("\nmoving backwards\n")
            m.move("backward")
        elif selection == "r":
            print("\nturning right\n")
            m.move("right")
        elif selection == "l":
            print("\nturning left\n")
            m.move("left")
        elif selection == "rr":
            print("\nrotating right in place\n")
            m.rotate("right", 5.0)
        elif selection == "ll":
            print("\nrotating left in place\n")
            m.rotate("left", 5.0)
        elif selection == "s":
            print("\nstopping\n")
            m.stop()
        elif selection == "e":
            print("\nexiting\n")
            break

if __name__ == "__main__":

    main()