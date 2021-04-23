#! /usr/bin/python

import datetime
import numpy as np
import rospy
import time

from competition2.srv import BanditStep, BanditAnswer


class UCBBanditRoom:

    def __init__(self):

        rospy.wait_for_service("/bandit_step")
        self.bandit_step_client = rospy.ServiceProxy("/bandit_step", BanditStep)

        rospy.wait_for_service("/bandit_answer")
        self.bandit_answer_client = rospy.ServiceProxy("/bandit_answer", BanditAnswer)

    def run_algorithm(self, passcode, num_arms):
        """

        @param passcode: int
        @param num_arms: int
        """

        print("\n\nlearning starting")

        t = 0
        c = 0.5

        q = np.zeros(num_arms)
        n = np.zeros(num_arms)
        u = np.zeros(num_arms)

        array_len = 6 * num_arms
        delta_qs = np.ones(array_len)

        def compute_ucb():
            """
            Compute upper confidence bound.
            """

            for a in range(num_arms):

                if n[a] == 0:
                    u[a] = float("inf")
                else:
                    u[a] = c * np.sqrt(np.log(t) / n[a])

        def policy():
            """
            Bandit agent policy.
            """

            compute_ucb()

            a = np.random.choice(np.where((q + u) == np.max((q + u)))[0])

            return a

        while t < 100 or np.min(u) > 0.05:

            action = policy()

            response = self.bandit_step_client(passcode, action+1) # must increment since actions are >= 1
            reward = response.reward
            valid = response.valid

            if valid == False:
                print("ERROR: invalid passcode or num_actions")
                return None, None

            n[action] += 1

            delta_q = (1. / n[action]) * (reward - q[action])
            delta_qs[t % (array_len)] = delta_q

            q[action] += delta_q

            t += 1

        print("\nlearning complete.  timesteps: {}.\n".format(t))

        arm = np.where(q == np.max(q))[0]

        if len(arm) > 1:
            print("\nERROR: more than one best arm\n")

        response = self.bandit_answer_client(arm[0]+1)

        room = response.room
        where = response.where

        print("\nresponse from server:")
        print("room {}".format(room))
        print("where {}\n".format(where))

        print("\norder of actions (from best action to worst action):")
        q_copy = np.copy(q).tolist()
        a = 0
        while a < num_arms:
            max_ = max(q_copy)
            max_index = q_copy.index(max_)
            q_copy[max_index] = -1
            print("action {}".format(max_index+1))
            a += 1
        print("\n")

        return room, where
