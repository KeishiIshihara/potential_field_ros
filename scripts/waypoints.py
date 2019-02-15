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


###################################################################################
## <Discription>
## This code is not visualizing the potential fields neither attractive and repusives.
## Computing the force that the robot recieves and visualize waypoints.
###################################################################################

class ComputeWaypoints:
    def __init__(self, grid):
        """
        Now you have to define 4 parameters as following: 
            self.width, self.height - cells
            self.resolution - resolution
            self.num_humans - number of humans
            self.dest1_x, self.dest2_y - destination of the robot
        """

        """ These variables are for the Map """
        self.width = grid.width #[cells]
        self.height = grid.height #[cells]
        self.resolution = grid.resolution #[m/cells]

        """ About waypoints """
        self.start_position_x = grid.start_position_x #299 # int(random.uniform(0, self.width)) # 240
        self.start_position_y = grid.start_position_y #200 # int(random.uniform(0, self.width)) # 240
        self.waypoints = []
        self.waypoints.append([self.start_position_x, self.start_position_y])

        """ Assume a destination of robots """
        self.dests = grid.dests

        """ About a human """
        self.humans = grid.humans
        self.human_vels = grid.human_vels

        """ These are coefficients to calculate the potential """
        self.kappa = grid.kappa #0.5 #3.0
        self.alpha = grid.alpha #150.0 #300.0 #20000 #700
        self.beta = grid.beta #1.0
        self.sigma = grid.sigma #1.0 #[m] #defalt is 1.5
        self.gamma = grid.gamma #30. #It seems like 25. is better for now
        self.kappa_att = grid.kappa_att #1.5
        self.delta = grid.delta #0.3

        self.epsilon = grid.epsilon # amount of movement
        self.zeta = grid.zeta #0.2 #[m] # Threshold of the distance from robot to the goal

        self.mu = 0. # degree of the velocity of a human
        self.x_h = [] #the Unit is [m] not pixels
        self.y_h = []
        self.d_h = []
        self.theta_h = []
        self.robot_dn_x = None #the Unit is [m] not pixels
        self.robot_dn_y = None

        self.rep_forces = []
        self.att_force = None

        """ To check the params """
        for i in range(len(self.humans)):
            if self.humans[i][0] >= self.width or self.humans[i][1] >= self.height:
                print "Error! : The human["+str(i+1)+"] is out of the map currently"
                sys.exit(1)


    def show_waypoints(self):
        potential_array = np.zeros((self.width,self.height),np.int) 
        
        x = np.linspace(0, self.width*self.resolution, self.width)
        y = np.linspace(0, self.height*self.resolution, self.height)
        X, Y = np.meshgrid(x, y)

        print "--------------- Map meta data -----------------"
        print "Width = "+str(self.width)+"[cells]"
        print "Height = "+str(self.height)+"[cells]"
        print "Resolution: "+str(self.resolution)+"[m/cells]"
        print "Domain of potential field = "+str(self.width*self.resolution)+"x"+str(self.height*self.resolution)+"[m2]"
        print "Destination of the robots = ("+str(self.dests[0][0])+","+str(self.dests[0][1])+")"
        print "-----------------------------------------------"

        self.set_robot_destination(self.dests[0][0]*self.resolution, self.dests[0][1]*self.resolution)
        checks = 0
        self.count = 0
        
        distance_from_robot_to_goal , theta = self.get_distance_to_the_destination(self.waypoints[checks][0]*self.resolution,self.waypoints[checks][1]*self.resolution)
        robot_gets_stuck = False
 
        while(distance_from_robot_to_goal > self.zeta and robot_gets_stuck == False):
            print ""
            print "--------------- loop "+str(checks+1)+" --------------------"
            ## Attractive Force
            coef_att, Fx_a, Fy_a, d, theta= self.get_attractive_force(self.waypoints[checks][0]*self.resolution, self.waypoints[checks][1]*self.resolution)

            # plt.quiver(self.waypoints[checks][1]*self.resolution, self.waypoints[checks][0]*self.resolution, Fx_a, Fy_a, color='r', angles='xy',scale_units='xy',scale=1)
            # plt.draw()

            self.att_force = [Fx_a, Fy_a]
            self.rep_forces = [] #just initialize every humans

            for h in range(len(self.humans)):
                potential_array[self.humans[h][0]][self.humans[h][1]] = 0.
                
                ## get Polar coordinate when you just put (x, y)
                d_h, theta_h = self.get_polar_coordinate_from_origin(self.humans[h][0]*self.resolution, self.humans[h][1]*self.resolution)

                ## set human position in both coordinate
                self.set_human_position(d_h, theta_h, self.humans[h][0]*self.resolution, self.humans[h][1]*self.resolution)

                ## Repulsive Force
                Fx_r, Fy_r, F_r, coef_rep = self.get_repulsive_force(self.waypoints[checks][0]*self.resolution, self.waypoints[checks][1]*self.resolution, h)

                # plt.quiver(self.waypoints[checks][1]*self.resolution, self.waypoints[checks][0]*self.resolution, Fy_r, Fx_r, color='b', angles='xy',scale_units='xy',scale=1)
                # plt.draw()
                self.rep_forces.append([Fy_r, Fx_r])
                plt.quiver(self.humans[h][1]*self.resolution,self.humans[h][0]*self.resolution,self.human_vels[h][1],self.human_vels[h][0], angles='xy',scale_units='xy',scale=1)
                plt.draw()

                # print ""
                # print "-------------  Num of humans: "+str(h+1)+" / "+str(len(self.humans))+" ------------"
                # print "human position: "
                # print "(x, y) = ("+str(self.x_h[h])+"[m], "+str(self.y_h[h])+"[m]) "
                # print "(r, theta) = ("+str(self.d_h[h])+"[m], "+str(round(math.degrees(self.theta_h[h]),2))+"[degree])"
                # print "human's velocity(direction) : "
                # print "(hVx, hVy) = ("+str(self.human_vels[h][0])+", "+str(self.human_vels[h][1])+")"
                # print "human direction = arctan(hVy/hVx) = "+str(round(math.degrees(math.atan2(self.human_vels[h][1], self.human_vels[h][0])),2))+"[degree])"
                # print "------------------------------------------------"

            ### Calculate the total force that the robot receves
            ### TODO: move these to new function: -> completed!
            total_force = self.visualize_total_force(checks)
            movement_x = int(Decimal((total_force[0]/self.resolution)).quantize(Decimal('0'), rounding=ROUND_HALF_UP))
            movement_y = int(Decimal((total_force[1]/self.resolution)).quantize(Decimal('0'), rounding=ROUND_HALF_UP))
            movement_x = int(self.epsilon * movement_x)
            movement_y = int(self.epsilon * movement_y)
            

            movement = np.array([movement_x, movement_y])
            movement = movement / np.linalg.norm(movement) * 5

            movement_x = movement[0]
            movement_y = movement[1]

            amount_of_movement = math.sqrt(movement_x**2+movement_y**2)

            print "movement vector = ("+str(movement_x)+", "+str(movement_y)+")"
            print "amount of movement = "+str(amount_of_movement)

            new_check_point_x = self.waypoints[checks][0] + movement_y
            new_check_point_y = self.waypoints[checks][1] + movement_x
            self.waypoints.append([new_check_point_x,new_check_point_y])
            print "current waypoints = "+str(self.waypoints[checks])
            plt.quiver(self.waypoints[checks][1]*self.resolution, self.waypoints[checks][0]*self.resolution, movement_x*self.resolution, movement_y*self.resolution, color='g', angles='xy',scale_units='xy',scale=1)
            plt.draw()
            checks += 1

            distance_from_robot_to_goal , theta = self.get_distance_to_the_destination(self.waypoints[checks][0]*self.resolution,self.waypoints[checks][1]*self.resolution)
            print "distance to the goal = "+str(distance_from_robot_to_goal)

            ### Errors >>
            if amount_of_movement == 0.0:
                robot_gets_stuck = True
            
            if checks > 500:
                robot_gets_stuck = True
            ### loops end here ###
        
        print ""
        if distance_from_robot_to_goal <= self.zeta and robot_gets_stuck == False:
            print "[finish] I achived to the goal finally!! "

        elif robot_gets_stuck == True and amount_of_movement == 0.0:
            print "[Error] The robot got stuck!! It is not able to move any more."
            if distance_from_robot_to_goal < 0.5:
                print "        But I might be almost around the goal."

        elif robot_gets_stuck == True and checks > 500:
            print "[Error] I can't get the goal. It seems something strange..."
            print "        Because the human's vectol is probably so large that I couldn't go to the goal."

        print "# Started point = ("+str(self.start_position_x)+", "+str(self.start_position_y)+")"
        print "# Goal point = ("+str(self.dests[0][0])+", "+str(self.dests[0][1])+")"

        ## show the potential_array(plt)
        # plt.contour(X, Y, potential_array, 20, cmap='viridis')
        # plt.colorbar()
 
        ax = plt.subplot()
        ax.plot()
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        

        # make nd.array of waypoints to use numpy functions
        np_waypoints = np.array(self.waypoints)

        plt.scatter(self.dests[0][1]*self.resolution, self.dests[0][0]*self.resolution, marker='o')

        plt.axis('equal')
        # plt.show()
        plt.pause(1)


    def visualize_total_force(self, checks):
        sum_rep_x = 0.
        sum_rep_y = 0.
        rep_force = None
        ## um all repulsives
        for i in range(len(self.rep_forces)):
            sum_rep_x += self.rep_forces[i][0]
            sum_rep_y += self.rep_forces[i][1]
        rep_force = [sum_rep_x, sum_rep_y]
        magnitude_rep_force = math.sqrt(rep_force[0]**2+rep_force[1]**2)

        # if magnitude_rep_force < 0.15: ## ignore total_force when it is too small
        #     rep_force = [0,0]
            
        # elif magnitude_rep_force  > 0.8: ## in case of that total_force is too large
        #     rep_force[0] = rep_force[0] / magnitude_rep_force
        #     rep_force[1] = rep_force[1] / magnitude_rep_force
        #     self.count += 1
           
        ## sum self.att_force and repulsive forces
        total_force = [(self.att_force[0]+rep_force[0]), (self.att_force[1]+rep_force[1])]

        # plt.quiver(self.waypoints[checks][1]*self.resolution, self.waypoints[checks][0]*self.resolution, total_force[0], total_force[1], color='g', angles='xy',scale_units='xy',scale=1)
        # plt.draw()

        return total_force


    def get_polar_coordinate_from_origin(self, x, y):
        x = float(x)
        d = math.sqrt((x**2 + ((self.height)*self.resolution-y)**2))
        theta = math.atan2(((self.height)*self.resolution-y) , x)

        return d, theta
    
    def set_human_position(self, d, theta, x, y):
        self.d_h.append(d) # [m]
        self.theta_h.append(theta) # [rad]
        self.x_h.append(x)
        self.y_h.append(y)
    
    def set_robot_destination(self, x, y):
        self.robot_dn_x = x
        self.robot_dn_y = y

    def get_relative_position(self, x_r, y_r, h):
        x_r = float(x_r)
        y_r = float(y_r)
        r = math.sqrt(((x_r - self.x_h[h])**2 + (y_r - self.y_h[h])**2))
        theta = math.atan2((y_r - self.y_h[h]) , (x_r - self.x_h[h]))
        return r, theta 
    
    def get_distance_to_the_destination(self, x_r, y_r):
        r = math.sqrt(((x_r - self.robot_dn_x)**2 + (y_r - self.robot_dn_y)**2))
        # theta = math.atan2( (x_r - self.robot_dn_x), (y_r - self.robot_dn_y) )
        theta = math.atan2( (self.robot_dn_x - x_r), (self.robot_dn_y - y_r) )

        return r, theta #Actually, it need to return only r

    
    def get_attractive_force(self, x_r, y_r): 
        d_hr, theta_hr = self.get_distance_to_the_destination(x_r, y_r)
        coef_att = self.kappa_att * d_hr
        # Fx = self.delta * coef_att * math.cos(theta_hr)
        # Fy = self.delta * coef_att * math.sin(theta_hr)
        Fx = self.delta * self.kappa_att * math.cos(theta_hr)
        Fy = self.delta * self.kappa_att * math.sin(theta_hr)
        return  coef_att, Fx, Fy, d_hr, theta_hr

    def get_repulsive_force(self, x_r, y_r, h):

        d_hr, theta_hr = self.get_relative_position(x_r, y_r, h)
        mu = math.atan2(self.human_vels[h][1], self.human_vels[h][0])
        coef = self.coefficientRisk(d_hr, theta_hr, self.human_vels[h][0], self.human_vels[h][1], self.kappa, mu)
        Fx = self.delta * self.vectorRiskX(d_hr, theta_hr, self.kappa, coef, mu)
        Fy = self.delta * self.vectorRiskY(d_hr, theta_hr, self.kappa, coef, mu)
        F = math.sqrt(Fx**2 + Fy**2)
        return Fx, Fy, F, coef

    def Besseli0(self, kappa):
        ax = abs(kappa)

        if ax < 3.75:
            y = kappa/3.75
            y = y**2
            ans = 1.0 + y*(3.5156229 + y*(3.0899424 + y*(1.2067492+ y*(0.2659732 + y*(0.360768e-1 + y*0.45813e-2)))))
        
        else:
            y = 3.75/ax
            ans = (math.exp(ax)/math.sqrt(ax)) * (0.39894228 + y*(0.1328592e-1 + y*(0.225319e-2 + y*(-0.157565e-2 + y*(0.916281e-2
                    + y*(-0.2057706e-1 + y*(0.2635537e-1 + y*(-0.1647633e-1 + y*0.392377e-2))))))))
        
        return ans

    def coefficientRisk(self, r, theta, hVx, hVy, kappa, mu):
        I = self.Besseli0(kappa)
        hV = math.sqrt(hVx**2 + hVy**2)
        coef = (self.alpha*self.beta*hV*math.exp(kappa*math.cos(theta-mu) - r/(2*self.sigma))) / (4*(math.pi*math.pi)*I*self.sigma)
        return coef 
    
    def vectorRiskX(self, r, theta, kappa, coefficient, mu):
        Fx = -1.0 * coefficient * (-1 * math.cos(theta)/(2*self.sigma) + kappa*math.sin(theta)*math.sin(theta-mu)/r)
        return Fx
    
    def vectorRiskY(self, r, theta, kappa, coefficient, mu):
        Fy = -1.0 * coefficient * (-1 * math.sin(theta)/(2*self.sigma) - kappa*math.cos(theta)*math.sin(theta-mu)/r)
        return Fy
    
def main():
    cw = ComputeWaypoints()
    cw.show_waypoints()


if __name__ == '__main__':
    main()

