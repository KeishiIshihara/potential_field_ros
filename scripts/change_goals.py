#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import sys
import cv2
import math
import signal
import random
from decimal import Decimal, ROUND_HALF_UP, ROUND_HALF_EVEN

from waypoints import ComputeWaypoints
from visualize_potentials import BehaviorPotentialField
from time import sleep

class ChangeGoals:
    def __init__(self):
        """ Meta difinition """
        self.width = 400 #[cells]
        self.height = 400 #[cells]
        self.resolution = 0.01 #[m/cells]

        self.start_position_x = 299 # int(random.uniform(0, self.width)) # 240
        self.start_position_y = 200 # int(random.uniform(0, self.width)) # 240

        self.dests = []
        self.dest1_x = 10
        self.dest1_y = 230

        # self.dest1_x = int(random.uniform(0, self.width)) 
        # self.dest1_y = int(random.uniform(0, self.width)) 
        self.dests.append([self.dest1_x,self.dest1_y])

        """ About a human """
        self.humans = []
        self.human_vels = []

        #### This moment, you can choose how to define people.
        #### Randomly and Manually are available.
        ## Make human positions and velocitys munually.   
        ## TODO: these params have to decided randamly.
        ## human 1
        self.humans.append([170,200]) ## position
        self.human_vels.append([0.5,0.5]) ## direction (velocity)
        ## human 2
        self.humans.append([180,100])
        self.human_vels.append([0.5,0.5])
        self.num_humans = len(self.humans)
        
        ## Make human positions and velocitys randomly.
        ## All you have to do is only to set number of humans.
        # self.num_humans = 2
        # for i in range(self.num_humans):
        #     ## human position
        #     humans_rand_x = int(random.uniform(0, self.width))
        #     humans_rand_y = int(random.uniform(0, self.height))
        #     self.humans.append([humans_rand_x, humans_rand_y])
        #     ## human velocity
        #     threshold_of_the_size_of_velocity = False
        #     count = 0
        #     while threshold_of_the_size_of_velocity is False:
        #         vel_rand = np.random.randint(-6, 6, 2).astype(float) / 10.
        #         r = math.sqrt(vel_rand[0]**2+vel_rand[1]**2)
        #         if r > 0.5:
        #             threshold_of_the_size_of_velocity = True
        #         count += 1
        #     self.human_vels.append(vel_rand.tolist())
        #     # print "# how many loops to decide a vel "+str(count)+" of human "+str(i)


        """ These are coefficients to calculate the potential """
        self.kappa = 0.5 #3.0
        self.alpha = 150.0 #300.0 #20000 #700
        self.beta = 1.0
        self.sigma = 1.0 #[m] #defalt is 1.5
        self.gamma = 30. #It seems like 25. is better for now

        self.kappa_att = 1.8 # gradient of the attractive force
        self.delta = 0.3

        self.epsilon = 0.2 # amount of movement
        self.zeta = 0.1 #[m] # Threshold of the distance from robot to the goal
        
        self.waypoints = ComputeWaypoints(self)
        self.pf = BehaviorPotentialField(self)
        

    def reset_goal_position(self, x, y):
        # reset new goal
        self.dests = []
        self.dests.append([x, y])
        self.waypoints.dests = self.dests
        last_point = self.waypoints.waypoints[-1]
        self.waypoints.waypoints = []
        self.waypoints.waypoints.append(last_point)
        self.pf.dests = self.dests

        # check if goal is human or not
        np_humans = np.array(self.humans)
        xy = np.array([x,y])
        goal_is_human = False
        for i in range(len(np_humans)):
            if (np_humans[i] == xy).all() == True:
                print np_humans[i]
                goal_is_human = True
                human_position = i

        # update humans if goal is human
        if goal_is_human == True:
            new_humans = []
            for i in range(len(self.humans)):
                if i != human_position:
                    new_humans.append(self.humans[i])

            ## Update the humans of the waypoints object and also self.humans of this class' object
            self.waypoints.humans = new_humans
            self.pf.humans = new_humans
            self.humans = new_humans

        # sys.exit(0)

    def show_waypoints(self):
        self.waypoints.show_waypoints()
        self.pf.show_bpf()

        reset_goal = True

        while reset_goal is True:
            print " --------- ------- -------- ------- ------- "
            print " --------- ------- -------- ------- ------- "
            print "Do you want to change the Goal ?"
            print "Range is in ("+str(self.width)+", "+str(self.height)+")"

            for i in range(len(self.humans)):
                print "  - human "+str(i)+" : ("+str(self.humans[i][0])+", "+str(self.humans[i][1])+")"
            
            # x, y = (int(x) for x in input("input goal position as (x,y) >> ").split())
            x, y = map(int, raw_input("input goal position as (x,y) >> ").split())
            print x, y
            
            # if 0 < (num_of_human) < len(self.humans)+1:
            if x < self.width and y < self.height:

                print "Now I'm gonna go to ("+str(x)+", "+str(y)+")"
                self.reset_goal_position(x, y)
                self.waypoints.show_waypoints()

            else:
                reset_goal = False
                sleep(5)
        
        print "Done."


def main():
    cg = ChangeGoals()
    cg.show_waypoints()


if __name__ == '__main__':
    main()