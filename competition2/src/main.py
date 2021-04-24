#! /usr/bin/python
import time
import rospy
import datetime
import rosparam
import read_room_number
import shapes_room
import yaml
import stitch_image
import read_clue as clue
import os
import constants

from pathlib import Path

from move import Move
from send_goal_client import SendGoalClient
from bandit_room import BanditRoom
from ucb_bandit_room import UCBBanditRoom


def lobby():
    """
    Lobby.

    @return next_room: int
    """

    start = time.time()

    #### add here ####
    # TODO read clue on the wall and save as next_room
    def read_clue():
        # stitchImage = stitch_image.StitchImage()
        # stitchImage.DEBUG = True
        # stitchImage.preStitch()
        # stitchImage.stitch()
        print("=== Report === Read from png")
        readClue = clue.ReadClue()
        next_room = readClue.lobby() # replace this line
        return next_room

    next_room = read_clue()
    while input("\n\nThe clue reads {}.  Is this correct?\n".format(next_room)) == "n":
        if input("\n\nWould you like to attempt to use opencv to read the clue again\n?") == "y":
            next_room = read_clue()
        else:
            print("\n\nManually obtaining the clue from the parameter server.")
            next_room = rosparam.get_param("/competition2_server/shapes_room")
            print("Shapes room is room {}.".format(next_room))
            

    # read map on the wall and associate numbers with letters
    # assumption: saved as dictionary (see numbered_locations.yaml for format)
    # assumption: lobby is assinged a value of 0
    def read_map():
        """
        Extract room numbers from the map.

        @return numbered_locations: dict {int: string}
        """
        numbered_locations = rosparam.load_file(str(Path.home()) + "/catkin_ws/src/competition2/yaml/numbered_locations.yaml")[0][0]
        return numbered_locations

    numbered_locations = read_map()
    print("\n\nroom assignments:")
    print(numbered_locations, "\n")

    while input("\nAre the numbered room assignments correct?\n") == "n":
        if input("\n\nWould you like to attempt to use opencv to read the map again?\n") == "y":
            numbered_locations = read_map()
        else:
            input_locations = input("\n\nManually enter the locations.  Please enter them in the from '1a,2b,3c,4d,...', with no spacing.\n")
            input_locations = input_locations.split(",")
            numbered_locations = {}
            for location in input_locations:
                if location == "0lobby":
                    number = 0
                    letter = "lobby"
                else:
                    number = int(location[:-1])
                    letter = location[-1]
                numbered_locations[number] = letter
        print("\n\nroom assignments:")
        print(numbered_locations, "\n")

    # save to file
    with open(str(Path.home()) + "/catkin_ws/src/competition2/yaml/numbered_locations.yaml", "w") as f:
        yaml.dump(numbered_locations, f)

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
    print("\n\ntime to complete lobby: {} h:mm:ss\n".format(completion_time))

    return next_room

def shapes():
    """
    Shapes room.

    @return next_room: int
    """

    start = time.time()
    
    # stitchImage = stitch_image.StitchImage()
    # stitchImage.DEBUG = True
    # stitchImage.preStitch()
    # stitchImage.stitch()
    print("=== Report === Read from png")
    readClue = clue.ReadClue()
    next_room = readClue.shapes() # replace this line
    
    #### add here ####
    shapesRoom = shapes_room.ShapesRoom()
    shapesRoom.identify_shape(next_room[0], next_room[1])
    #### spin ####
    shapesRoom.getResult()
    #### do somethin with next_room ####
    count = shapesRoom.getResult()
    next_room = 19
    what = "frying pan"  # TODO remove


    #### end here ####

    end = time.time()
    completion_time = str(datetime.timedelta(seconds=(end - start))).split(".")[0]
    print("\ntime to complete shapes room: {} h:mm:ss\n".format(completion_time))

    return next_room, what

def bandits():
    """
    Bandits room.

    @return next_room: int
    """

    start = time.time()

    #### add here ####

    def read_clue():

        passcode = 42
        num_arms = 8

        # TODO read passcode and num_arms from the clue
        # passcode [1,99]
        # num_arms [2,8]
        # stitchImage = stitch_image.StitchImage()
        # stitchImage.DEBUG = True
        # stitchImage.preStitch()
        # stitchImage.stitch()
        print("=== Report === Read from png")
        readClue = clue.ReadClue()
        next_room = readClue.bandit() # replace this line    
        if len(next_room) == 2:
            passcode = next_room[0]
            num_arms = next_room[1]
        return passcode, num_arms

    passcode, num_arms = read_clue()

    # check input
    while input("\n\nThe clue reads: passcode {}, num_rooms {}.  Is this correct?\n".format(passcode, num_arms)) == "n":
        if input("\n\nWould you like to attempt to use opencv to read the clue again?\n") == "y":
            passcode, num_arms = read_clue()
        else:
            print("\n\nManually obtaining the clue from the parameter server.")
            passcode = rosparam.get_param("/competition2_server/bandit_passcode")
            num_arms = rosparam.get_param("/competition2_server/bandit_num_arms")
            print("Passcode is {}.  Number of arms is {}.".format(passcode, num_arms))

    next_room, where = bandit_room.run_algorithm(passcode, num_arms)

    #### end here ####

    end = time.time()
    completion_time = str(datetime.timedelta(seconds=(end - start))).split(".")[0]
    print("\ntime to complete bandit room: {} h:mm:ss\n".format(completion_time))

    return next_room, where

def maze():
    """
    Maze room.

    @return next_room: int
    """

    start = time.time()

    #### add here ####

    next_room = 0 # TODO remove
    who = "Joe" # TODO remove


    #### end here ####

    end = time.time()
    completion_time = str(datetime.timedelta(seconds=(end - start))).split(".")[0]
    print("\ntime to complete maze room: {} h:mm:ss\n".format(completion_time))

    return next_room, who

if __name__ == "__main__":
    print("=== Detect Tesseract ===")
    os.system("./install.sh")
    print("=== Preparing ===")
    print("=== Removing previous room number correspondence ===")
    os.system("rm " + constants.NUMBERED_LOCATIONS)
    print("=== Removing previous stitched images ===")
    os.system("rm -rf " + constants.STITCH_IMAGE_COMMON_PATH_PREFIX[:-1])
    os.system("mkdir -p " + constants.STITCH_IMAGE_COMMON_PATH_PREFIX[:-1])
    print("=== Prepare complete ===")
    rospy.init_node("main_node")

    # start program
    print("\nstarting program\n")
    main_start = time.time()

    readRoomNumber = read_room_number.ReadRoomNumber()
    readRoomNumber.readRoomNumber(readRoomNumber.readPath)
    readRoomNumber.writeYaml()

    # initialize classes
    bandit_room = UCBBanditRoom()
    move = Move()
    send_goal_client = SendGoalClient()

    # clue
    who = "Joe"
    where = "kitchen"
    what = "frying pan"

    # localize
    send_goal_client.localize()

    # lobby
    send_goal_client.traverse(0)
    while input("\n\nWould you like to start the lobby task?\n") == "y":
        next_room = lobby()

    # shape room
    send_goal_client.traverse(next_room)
    while input("\n\nWould you like to start the shapes task?\n") == "y":
        next_room, what = shapes()

    # bandit room
    send_goal_client.traverse(next_room)
    while input("\n\nWould you like to start the bandits task?\n") == "y":
        next_room, where = bandits()

    # maze room
    send_goal_client.traverse(next_room)
    while input("\n\nWould you like to start the maze room?\n") == "y":
        next_room, who = maze()

    # solution reading room
    # send_goal_client.traverse(next_room)
    print("\n\nfinal solution:")
    print("who:", who)
    print("what:", what)
    print("where:", where)

    # program end time
    print("\n\nending program\n")
    main_end = time.time()

    completion_time = str(datetime.timedelta(seconds=(main_end - main_start))).split(".")[0]
    print("\n\ntime to complete program: {} h:mm:ss\n".format(completion_time))
