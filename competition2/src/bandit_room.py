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

        self.epsilon = 0.1
        self.t = 0

    def init_algorithm(self, passcode, num_arms): # TODO
        """
        Save the passcode and num_arms.

        @param passcode: int
        @param num_arms: int
        """

        self.passcode = passcode
        self.num_arms = num_arms

        self.q = np.zeros(self.num_arms)
        self.n = np.zeros(self.num_arms)

    def run_algorithm(self):

        delta_qs = np.zeros(20)

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
        while np.max(delta_qs) > 0.02:

            action = policy()

            response = self.bandit_step_client(self.passcode, action+1) # must increment since actions are >= 1
            reward = response.reward
            valid = response.valid


            self.n[action] += 1

            delta_q = (1. / self.n[action]) * (reward - self.q[action])
            delta_qs[self.t % 20] = delta_q

            self.q[action] += delta_q

            self.t += 1

        print("\nlearning complete.  timesteps: {}.\n".format(self.t))

    def close_algorithm(self):
        """
        Submit the answer.
        """

        arm = np.where(self.q == np.max(self.q))[0]

        while len(arm) > 1:
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


# def main():
#
#     rospy.init_node("bandit_problem_node")
#
#     try:
#
#         br = BanditRoom()
#         br.init_algorithm()
#         br.run_algorithm()
#         br.close_algorithm()
#
#     except rospy.ServiceException as e:
#
#         print(e)
#
#     end = time.time()
#
#     completion_time = end - start
#     completion_time = str(datetime.timedelta(seconds=completion_time)).split(".")[0]
#
#     print("\ntime to complete bandits room: {} h:mm:ss".format(completion_time))
#
#
# if __name__ == "__main__":
#
#     main()
