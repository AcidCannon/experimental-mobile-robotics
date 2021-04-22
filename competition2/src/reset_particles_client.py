#! /usr/bin/python

import rospy
from std_srvs.srv import Empty, EmptyRequest

class ResetParitclesClient:
    """
    amcl node must be running for this to work
    """

    def __init__(self):

        rospy.wait_for_service("/global_localization")

        try:

            service_client = rospy.ServiceProxy("/global_localization", Empty)

            empty_request = EmptyRequest()

            service_client(empty_request)

        except rospy.ServiceException as e:

            print(e)

def main():

    rospy.init_node("reset_particles_client_node")

    ResetParitclesClient()

if __name__ == "__main__":

    main()