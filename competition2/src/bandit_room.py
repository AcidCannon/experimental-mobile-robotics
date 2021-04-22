#! /usr/bin/python

import datetime
import numpy as np
import rospy
import time

from competition2.srv import BanditStep, BanditAnswer


class BanditRoom:

    def __init__(self):

        rospy.wait_for_service("/bandit_step")
        self.bandit_step_client = rospy.ServiceProxy("/bandit_step", BanditStep)

        rospy.wait_for_service("/bandit_answer")
        self.bandit_answer_client = rospy.ServiceProxy("/bandit_answer", BanditAnswer)

        self.passcode = None
        self.num_arms = None

        self.q = None
        self.n = None

        self.epsilon = 0.05
        self.t = 0
        self.timesteps_per_arm = 5000

    def init_algorithm(self): # TODO
        """
        Save the passcode and num_arms.
        """

        # print("\ninitializing algorithm")

        # need passcode
        self.passcode = int(input("\nEnter the passcode, then hit enter.\n"))

        # need num_arms
        self.num_arms = int(input("\nEnter the number of arms, then hit enter.\n"))

        self.q = np.zeros(self.num_arms)
        self.n = np.zeros(self.num_arms)

    def run_algorithm(self):

        # print("\nrunning algorithm")

        def policy():
            """
            Bandit agent policy.
            """

            if np.random.uniform() >= self.epsilon:

                # greedy action
                a = np.random.choice(np.where(self.q == np.max(self.q))[0])

            else:

                # random action
                a = np.random.choice(np.arange(self.num_arms))

            return a

        # problem complexity increases with the num_arms
        while self.t < (self.num_arms * self.timesteps_per_arm):

            action = policy() 

            response = self.bandit_step_client(self.passcode, action+1) # must increment since actions are >= 1
            reward = response.reward
            valid = response.valid

            # check to make sure passcode and action is valid; update it again
            attempt = 1
            max_attempts = 2
            while not valid and attempt <= max_attempts:
                print(attempt)
                print("\neither passcode or num_arms is invalid")
                print("attempt {} / {}".format(attempt, max_attempts))
                self.init_algorithm()
                action = policy()
                response = self.bandit_step_client(self.passcode, action+1)
                reward = response.reward
                valid = response.valid

                if attempt == max_attempts:
                    print("\nmax attempts reached")
                    return
                attempt += 1

            self.n[action] += 1
            self.q[action] += (1. / self.n[action]) * (reward - self.q[action])

            self.t += 1

    def close_algorithm(self):
        """
        Submit the answer.
        """

        # print("\nclosing algorithm")

        arm = np.where(self.q == np.max(self.q))[0]

        while len(arm) > 1:
            self.timesteps_per_arm += 100
            self.run_algorithm()

        response = self.bandit_answer_client(arm[0]+1)

        room = response.room
        where = response.where

        print("\nresponse from server:")
        print("room {}".format(room))
        print("where {}".format(where))

        print("\norder of actions (from best action to worst action):")
        q_copy = np.copy(self.q).tolist()
        a = 0
        while a < self.num_arms:
            max_ = max(q_copy)
            max_index = q_copy.index(max_)
            q_copy[max_index] = -1
            print("action {}".format(max_index+1))
            a += 1


def main():

    rospy.init_node("bandit_problem_node")

    try:

        br = BanditRoom()
        br.init_algorithm()
        br.run_algorithm()
        br.close_algorithm()

    except rospy.ServiceException as e:

        print(e)
    
    end = time.time()

    completion_time = end - start
    completion_time = str(datetime.timedelta(seconds=completion_time)).split(".")[0]

    print("\ntime to complete bandits room: {} h:mm:ss".format(completion_time))


if __name__ == "__main__":

    main()
