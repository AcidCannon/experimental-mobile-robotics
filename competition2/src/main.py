import time
import rospy
import datetime
import rosparam
import yaml

from move import Move
from send_goal_client import SendGoalClient
from bandit_room import BanditRoom


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
    while input("\nThe clue reads {}.  Is this correct?\n".format(next_room)) == "n":
        if input("\nWould you like to attempt to use opencv to read the clue again\n?") == "y":
            next_room = read_clue()
        else:
            next_room = input("\nManually enter the clue.  Please enter either 'highest' or 'lowest'.\n")

    # TODO read map on the wall and associate numbers with letters
    # assumption: saved as yaml named numbered_locations.yaml (see file for format)
    # assumption: lobby is assinged a value of 0
    def read_map():

        # add here


        # end here

        numbered_locations = rosparam.load_file("/home/user/catkin_ws/src/competition2/yaml/numbered_locations.yaml")[0][0]
        print("\nroom assignments:")
        print(numbered_locations, "\n")


    read_map()
    while input("\nAre the numbered room assignments correct?\n") == "n":
        if input("\nWould you like to attempt to use opencv to read the map again?\n") == "y":
            read_map()
        else:
            input_locations = input("\nManually enter the locations.  Please enter them in the from '1a,2b,3c,4d,...', with no spacing.\n")
            input_locations = input_locations.split(",")
            numbered_locations = {}
            for location in input_locations:
                if location == "0lobby":
                    number = 0
                    letter = "lobby"
                else:
                    number = location[:-1]
                    letter = location[-1]
                numbered_locations[number] = letter
            with open("/home/user/catkin_ws/src/competition2/yaml/numbered_locations.yaml", "w") as f:
                yaml.dump(numbered_locations, f)
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

    def read_clue():

        passcode = 42
        num_arms = 7

        # TODO read passcode and num_arms from the clue
        # passcode [1,99]
        # num_arms [2,8]

        return passcode, num_arms

    passcode, num_arms = read_clue()

    # check input
    while input("\nThe clue reads: passcode {}, num_rooms {}.  Is this correct?\n".format(passcode, num_arms)) == "n":
        if input("\nWould you like to attempt to use opencv to read the clue again?\n") == "y":
            passcode, num_arms = read_clue()
        else:
            passcode = int(input("\nManually enter the clue.  Please enter the passcode.\n"))
            num_arms = int(input("\nManually enter the clue.  Please enter the num_arms.\n"))

    bandit_room.init_algorithm(passcode, num_arms)
    bandit_room.run_algorithm()
    next_room, where = bandit_room.close_algorithm()

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
    bandit_room = BanditRoom()
    move = Move()
    send_goal_client = SendGoalClient()

    # localize
    # send_goal_client.localize()

    # lobby
    # send_goal_client.traverse(0)
    # next_room = lobby()

    # # shape room
    # send_goal_client.traverse(next_room)
    # next_room = shapes()

    # # bandit room
    # send_goal_client.traverse(next_room)
    next_room = bandits()

    # # maze room
    # send_goal_client.traverse(next_room)
    # next_room = maze()

    # program end time
    print("\nending program\n")
    main_end = time.time()

    completion_time = str(datetime.timedelta(seconds=(main_end - main_start))).split(".")[0]
    print("\ntime to complete program: {} h:mm:ss\n".format(completion_time))
