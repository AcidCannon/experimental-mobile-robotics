import time
import rospy
import datetime
import rosparam
import yaml

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
        next_room = "highest" # replace this line
        return next_room

    next_room = read_clue()
    while input("\n\nThe clue reads {}.  Is this correct?\n".format(next_room)) == "n":
        if input("\n\nWould you like to attempt to use opencv to read the clue again\n?") == "y":
            next_room = read_clue()
        else:
            print("\n\nManually obtaining the clue from the parameter server.")
            next_room = rosparam.get_param("/competition2_server/shapes_room")
            print("Shapes room is room {}.".format(next_room))
            

    # TODO read map on the wall and associate numbers with letters
    # assumption: saved as dictionary (see numbered_locations.yaml for format)
    # assumption: lobby is assinged a value of 0
    def read_map():
        """
        Extract room numbers from the map.

        @return numbered_locations: dict {int: string}
        """

        # add here


        # end here

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

    #### add here ####

    next_room = 0  # TODO remove
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

def test_traverse():

    send_goal_client.localize()
    # send_goal_client.traverse(0)
    send_goal_client.teleport(1)
    send_goal_client.localize()
    send_goal_client.traverse(0)
    send_goal_client.teleport(2)
    send_goal_client.localize()
    send_goal_client.traverse(1)
    # send_goal_client.traverse(3)

if __name__ == "__main__":

    rospy.init_node("main_node")

    # start program
    print("\nstarting program\n")
    main_start = time.time()

    # initialize classes
    bandit_room = UCBBanditRoom()
    move = Move()
    send_goal_client = SendGoalClient()

    # TODO remove if running
    test_traverse()

    # clue
    who = "Joe"
    where = "kitchen"
    what = "frying pan"

    # localize
    # send_goal_client.localize()

    # lobby
    # send_goal_client.traverse(0)
    # while input("\n\nWould you like to start the lobby task?\n") == "y":
    #     next_room = lobby()

    # shape room
    # send_goal_client.traverse(next_room)
    # while input("\n\nWould you like to start the shapes task?\n") == "y":
    #     next_room, what = shapes()

    # bandit room
    # send_goal_client.traverse(next_room)
    # while input("\n\nWould you like to start the bandits task?\n") == "y":
    #     next_room, where = bandits()

    # maze room
    # send_goal_client.traverse(next_room)
    # while input("\n\nWould you like to start the maze room?\n") == "y":
    #     next_room, who = maze()

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
