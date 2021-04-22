#! /usr/bin/python
import time
import rospy
import datetime
import rosparam
import read_room_number
import shape_room

from create_map_move import Move
from send_goal_client import SendGoalClient
from bandit_room import BanditRoom


def lobby():
    """
    Lobby.

    @return next_room: int
    """

    start = time.time()

    #### add here ####

    # read clue on the wall and get next room to move to
    # print 'highest' or 'lowest'
    # TODO
    next_room = "highest" # should be a string: 'highest' or 'lowest'
    print("\nclue: traverse to {} numbered room\n".format(next_room))

    # read map on the wall and associate numbers with letters
    # save to numbered_locations.yaml
    readRoomNumber = read_room_number.ReadRoomNumber()
    readRoomNumber.readRoomNumber(readRoomNumber.readPath)
    readRoomNumber.writeYaml()
    # assumption: saved as yaml named numbered_locations.yaml (see file for format)
    # assumption: lobby is assinged a value of 0
    
    # print out letter/number associations using images/figure5.jpg
    numbered_locations = rosparam.load_file("/home/user/catkin_ws/src/competition2/yaml/numbered_locations.yaml")[0][0]
    print("\nroom assignments:")
    print(numbered_locations, "\n")

    # reload yaml into send_goal_client class
    send_goal_client.numbered_locations = numbered_locations

    # determine next room using clue
    numbered_locations_keys = numbered_locations.keys()
    if next_room == "highest":
        next_room = max(numbered_locations)
    elif next_room == "lowest":
        next_room = min(numbered_locations)

    #### end here ####

    end = time.time()
    completion_time = str(datetime.timedelta(seconds=(end - start))).split(".")[0]
    print("\ntime to complete lobby: {} h:mm:ss\n".format(completion_time))

    return next_room

def shapes():
    """
    Shapes room.

    @return next_room: int
    """

    start = time.time()

    #### add here ####


    #### end here ####

    end = time.time()
    completion_time = str(datetime.timedelta(seconds=(end - start))).split(".")[0]
    print("\ntime to complete shapes room: {} h:mm:ss\n".format(completion_time))

    return next_room

def bandits():
    """
    Bandits room.

    @return next_room: int
    """

    start = time.time()

    #### add here ####


    #### end here ####

    end = time.time()
    completion_time = str(datetime.timedelta(seconds=(end - start))).split(".")[0]
    print("\ntime to complete bandit room: {} h:mm:ss\n".format(completion_time))

    return next_room

def maze():
    """
    Maze room.

    @return next_room: int
    """

    start = time.time()

    #### add here ####


    #### end here ####

    end = time.time()
    completion_time = str(datetime.timedelta(seconds=(end - start))).split(".")[0]
    print("\ntime to complete maze room: {} h:mm:ss\n".format(completion_time))

    return next_room



if __name__ == "__main__":

    rospy.init_node("main_node")

    # start program
    print("\nstarting program\n")
    main_start = time.time()

    # initialize classes
    move = Move()
    send_goal_client = SendGoalClient()
    bandit_room = BanditRoom()

    # localize
    send_goal_client.localize()

    # lobby
    send_goal_client.traverse(0)
    next_room = lobby()

    # shape room
    send_goal_client.traverse(next_room)
    next_room = shapes()

    # bandit room
    send_goal_client.traverse(next_room)
    next_room = bandits

    # maze room
    send_goal_client.traverse(next_room)
    next_room = maze()

    # program end time
    print("\nending program\n")
    main_end = time.time()

    completion_time = str(datetime.timedelta(seconds=(main_end - main_start))).split(".")[0]
    print("\ntime to complete program: {} h:mm:ss\n".format(completion_time))






