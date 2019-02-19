#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import sys
import cv2
import math
import signal
import random

#######################################################################
## <Discription>
## Visualize the potential fields both attractive and repusives.
## And also, compute the force that the robot recieves and visualize.
#######################################################################

class BehaviorPotentialField:
    def __init__(self, pose):
        """
        Now you have to define 4 parameters as following: 
            self.width, self.height - cells
            self.resolution - resolution
            self.num_humans - number of humans
            self.dest1_x, self.dest2_y - destination of the robot
        """

        """ These variables are for the Map """
        self.width = pose.grid.info.width #[cells]
        self.height = pose.grid.info.height #[cells]
        self.resolution = pose.grid.info.resolution #[m/cells]

        self.size_x = self.width*self.resolution
        self.size_y = self.height*self.resolution

        """ Check tools for the forces at one point """
        # self.check_x = 240
        # self.check_y = 240

        # self.rep_forces = []
        # self.att_force = None

        # self.scale = 0.15 # for visualize vectors

        """ Assume a destination of robots """
        self.dests = pose.dests

        """ About a human """
        self.humans = pose.humans
        self.human_vels = pose.human_vels


        """ These are coefficients to calculate the potential """
        self.kappa = pose.kappa #0.5 #3.0
        self.alpha = pose.alpha #150.0 #300.0 #20000 #700
        self.beta = pose.beta #1.0
        self.sigma = pose.sigma #1.0 #[m] #defalt is 1.5
        self.gamma = pose.gamma #30. #It seems like 25. is better for now

        self.kappa_att = pose.kappa_att #1.5
        # self.delta = pose.delta #0.3

        self.mu = 0. # degree of the velocity of a human
        self.x_h = [] #the Unit is [m] not pixels
        self.y_h = []
        self.d_h = []
        self.theta_h = []

        self.robot_dn_x = None #the Unit is [m] not pixels
        self.robot_dn_y = None

        """ To check the params """
        for i in range(len(self.humans)):
            if self.humans[i][0] >= self.width or self.humans[i][1] >= self.height:
                print "Error! : The human["+str(i+1)+"] is out of the map currently"
                sys.exit(1)


    def show_bpf(self):
        potential_array = np.zeros((self.width,self.height),np.int) 
        
        # x = np.linspace(0, self.size_x, self.width)
        x = np.linspace(-self.size_x/2, self.size_x/2, self.width)
        # y = np.linspace(0, self.size_y, self.height)
        y = np.linspace(-self.size_y/2, self.size_y/2, self.height)
        X, Y = np.meshgrid(x, y)

        print "--------------- Map meta data -----------------"
        print "Width = "+str(self.width)+"[cells]"
        print "Height = "+str(self.height)+"[cells]"
        print "Resolution: "+str(self.resolution)+"[m/cells]"
        print "Domain of potential field = "+str(self.size_x)+"x"+str(self.size_y)+"[m2]"
        print "Destination of the robots = ("+str(self.dests[0][0])+","+str(self.dests[0][1])+")"
        print "-----------------------------------------------"


        for h in range(len(self.humans)):
            potential_array[int(self.humans[h][0]/self.resolution)][int(self.humans[h][1]+self.width/2)] = 0.
            potential_array[self.dests[0][0]][self.dests[0][1]] = 0.
            
            ## get Polar coordinate when you just put (x, y)
            d_h, theta_h = self.get_polar_coordinate_from_origin(self.humans[h][0]/self.resolution, (self.humans[h][1]/self.resolution+self.width/2))

            ## set human position in both coordinate
            self.set_human_position(d_h, theta_h, self.humans[h][0]/self.resolution, (self.humans[h][1]+self.width/2)/self.resolution)
            self.set_robot_destination(self.dests[0][0]*self.resolution, self.dests[0][1]*self.resolution)


            ### Visualization for the both potential fields
            for i in range(potential_array.shape[0]):
                # print ""
                for j in range(potential_array.shape[1]):
                    if not (i == (self.humans[h][0]/self.resolution or self.dests[0][0]) and j == ((self.humans[h][1]/self.resolution+self.width/2) or self.dests[0][1])):
                        coef_att, Fx_a, Fy_a, d, theta= self.get_attractive_force(i*self.resolution, j*self.resolution, i, j)
                        coef_att = 0. #coef_att * self.gamma # = 0.


                        # if i == self.check_x and j == self.check_y:
                        #     plt.quiver(j*self.resolution, i*self.resolution, Fx_a, Fy_a, color='r', angles='xy',scale_units='xy',scale=1)
                        #     plt.draw()
                        #     F_a = math.sqrt(Fx_a**2+Fy_a**2)
                        #     self.att_force = [Fx_a, Fy_a]
                        #     print "------------ checking attractive forces -------------"
                        #     print "check point = ("+str(self.check_x)+", "+str(self.check_y)+")"
                        #     print "(Fx_a, Fy_a) = ("+str(Fx_a)+", "+str(Fy_a)+")"
                        #     print "F_a = "+str(F_a)
                        #     print "(d_hr, theta_hr) = ("+str(d)+", "+str(math.degrees(theta))+")"
                        #     print "------------------------------------------------------"
                        #     # if end //

                        
                        Fx_r, Fy_r, F_r, coef_rep = self.get_repulsive_force(i*self.resolution, j*self.resolution, h)
                        coef_rep = coef_rep*self.gamma

                        # if i == self.check_x and j == self.check_y:
                        #     plt.quiver(j*self.resolution, i*self.resolution, Fy_r, Fx_r, color='b', angles='xy',scale_units='xy',scale=1)
                        #     plt.draw()
                        #     F_r = math.sqrt(Fx_r**2+Fy_r**2)
                        #     self.rep_forces.append([Fy_r, Fx_r])
                        #     print "------------- checking repulsive forces --------------"
                        #     print "check point = ("+str(self.check_x)+", "+str(self.check_y)+")"
                        #     print "(Fx_r, Fy_r) = ("+str(Fx_r)+", "+str(Fy_r)+")"
                        #     print "F_r = "+str(F_r)
                        #     print "-----------------------------------------------------"
                        #     # if end //
   
                        # sys.stdout.write(str(coef) + " ")

                        ### Adopt the larger value (att or rep) at the point
                        coef = coef_att if coef_att > coef_rep else coef_rep

                        ### Adopt the larger value (potential_array or coef) at the point
                        if potential_array[i][j] < coef:
                            potential_array[i][j] = coef
            

            plt.quiver(self.humans[h][1]+(self.width/2)/self.resolution,self.humans[h][0], self.human_vels[h][1],self.human_vels[h][0], angles='xy',scale_units='xy',scale=1)
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
            

        # ### Calculate the total force that the robot receves
        # ### TODO: move these to new function
        # sum_rep_x = 0.
        # sum_rep_y = 0.
        # rep_force = None
        # ## sum all repulsives
        # for i in range(len(self.rep_forces)):
        #     sum_rep_x += self.rep_forces[i][0]
        #     sum_rep_y += self.rep_forces[i][1]
        # rep_force = [sum_rep_x, sum_rep_y]
        # ## sum self.att_force and repulsive forces
        # total_force = [self.att_force[0] + rep_force[0], self.att_force[1] + rep_force[1]]
        # ## draw
        # plt.quiver(self.check_x*self.resolution, self.check_y*self.resolution, total_force[0], total_force[1], color='g', angles='xy',scale_units='xy',scale=1)
        # plt.draw()

        ## show the potential_array(plt)
        plt.contour(X, Y, potential_array, 20, cmap='viridis')
        plt.colorbar()
        plt.pause(5)

        # sys.exit(0)


        # potential_array = potential_array.max() 
        # data = potential_array / potential_array.astype(float) * 100
        # data = list(potential_array.astype(int).flat)
        # return data

        # sys.exit(0)

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
    
    def get_distance_to_the_destination(self, x_r, y_r, i, j):
        r = math.sqrt(((x_r - self.robot_dn_x)**2 + (y_r - self.robot_dn_y)**2))
        # theta = math.atan2( (x_r - self.robot_dn_x), (y_r - self.robot_dn_y) )
        theta = math.atan2( (self.robot_dn_x - x_r), (self.robot_dn_y - y_r) )

        return r, theta #Actually, it need to return only r

    
    def get_attractive_force(self, x_r, y_r, i, j): 
        d_hr, theta_hr = self.get_distance_to_the_destination(x_r, y_r, i, j)
        coef_att = self.kappa_att * d_hr
        Fx = coef_att * math.cos(theta_hr)
        Fy = coef_att * math.sin(theta_hr)
        return  coef_att, Fx, Fy, d_hr, theta_hr

    def get_repulsive_force(self, x_r, y_r, h):

        d_hr, theta_hr = self.get_relative_position(x_r, y_r, h)
        mu = math.atan2(self.human_vels[h][1], self.human_vels[h][0])
        coef = self.coefficientRisk(d_hr, theta_hr, self.human_vels[h][0], self.human_vels[h][1], self.kappa, mu)
        Fx = self.vectorRiskX(d_hr, theta_hr, self.kappa, coef, mu)
        Fy = self.vectorRiskY(d_hr, theta_hr, self.kappa, coef, mu)
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
        
        # return ans
        return 1.

    def coefficientRisk(self, r, theta, hVx, hVy, kappa, mu):
        delta_theta = theta - mu
        # delta_theta = delta_theta if abs(delta_theta) < math.pi / 2 else math.pi / 2
        I = self.Besseli0(kappa)
        hV = math.sqrt(hVx**2 + hVy**2)
        coef = (self.alpha*self.beta*hV*math.exp(kappa*math.cos(delta_theta) - r/(2*self.sigma))) / (4*(math.pi*math.pi)*I*self.sigma)
        return coef 
    
    def vectorRiskX(self, r, theta, kappa, coefficient, mu):
        Fx = -1.0 * coefficient * (-1 * math.cos(theta)/(2*self.sigma) + kappa*math.sin(theta)*math.sin(theta-mu)/r)
        return Fx
    
    def vectorRiskY(self, r, theta, kappa, coefficient, mu):
        Fy = -1.0 * coefficient * (-1 * math.sin(theta)/(2*self.sigma) - kappa*math.cos(theta)*math.sin(theta-mu)/r)
        return Fy
    
def main():
    bP = BehaviorPotentialField()
    bP.show_bpf()


if __name__ == '__main__':
    main()

