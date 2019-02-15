#!/usr/bin/env python
#-*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import sys
import cv2
import math
import signal
# signal.signal(signal.SIGINT, signal.SIG_DFL)

class BehaviorPotentialField:
    def __init__(self, grid):
        """
        Now you have to define 3 parameters as following: 
            - self.n : number of grids
            - self.human1_x , self.human_y : human position
            - self.hVx, self.hVy : This is just used for defining human direction
        """

        """ These variables are for the Map """
        self.n = grid.info.width #=300 #[cells] 
        #TODO: self.n -> you must devide all self.n into grid.info.width & grid.info.height

        self.resolution = grid.info.resolution #=0.01 #[m/cells]

        self.size_x = self.n*self.resolution
        self.size_y = self.n*self.resolution

        """ Assume a destination of robots """
        self.dests = []
        # self.dest1_x = 260
        # self.dest1_y = 270
        self.dest1_x = 10
        self.dest1_y = 270
        self.dests.append([self.dest1_x,self.dest1_y])

        """ About a human """
        ## TODO: these params have to decided randamly.
        self.humans = []
        self.human_vels = []
        ## human 1
        self.humans.append([200,200]) ## position
        self.human_vels.append([-0.5,-0.5]) ## direction (velocity)
        ## human 2
        self.humans.append([180,100])
        self.human_vels.append([0.5,0.5])

        # print self.humans,len(self.humans)
        # print self.human_vels,len(self.human_vels)
        # sys.exit(0)

        """ These are coefficients to calculate the potential """
        self.kappa = 3. #3.0
        self.alpha = 150.0 #300.0 #20000 #700
        self.beta = 1.0
        self.sigma = 1.0 #[m] #defalt is 1.5
        self.gamma = 25.

        self.kappa_att = 2.5

        self.mu = 0. # degree of the velocity of a human
        self.x_h = [] #the Unit is [m] not pixels
        self.y_h = []
        self.d_h = []
        self.theta_h = []

        self.robot_dn_x = None #the Unit is [m] not pixels
        self.robot_dn_y = None


        """ To check the params """
        for i in range(len(self.humans)):
            if self.humans[i][0] >= self.n or self.humans[i][1] >= self.n:
                print "Error! : The human["+str(i+1)+"] is out of the map currently"
                sys.exit(1)


    def show_bpf(self):
        bpf_array = np.zeros((self.n,self.n),np.uint8) #np.uint8
        bpf_array_for_cv2 = np.zeros((self.n,self.n),np.uint8) #np.uint8 #Generate a brack image

        x = np.linspace(0, self.size_x, self.n)
        y = np.linspace(0, self.size_y, self.n)
        X, Y = np.meshgrid(x, y)
        # print type(X), X.shape

        print "--------------- Map meta data -----------------"

        print "Width = "+str(self.n)+"[cells]"
        print "Height = "+str(self.n)+"[cells]"
        print "Resolution: "+str(self.resolution)+"[m/cells]"
        print "Domain of potential field = "+str(self.size_x)+"x"+str(self.size_y)+"[m2]"

        for h in range(len(self.humans)):
            """
                self.humans[h][0] -> x of human No.h
                self.humans[h][1] -> y of human No.h
                self.humans_vels[h][0] -> Vx of human No.h
                self.humans_vels[h][1] -> Vy of human No.h
            """

            bpf_array[self.humans[h][0]][self.humans[h][1]] = 0.
            bpf_array[self.dests[0][0]][self.dests[0][1]] = 0.
            
            ## get Polar coordinate when you just put (x, y)
            d_h, theta_h = self.get_polar_coordinate_from_origin(self.humans[h][0]*self.resolution, self.humans[h][1]*self.resolution)
            # print math.degrees(theta_h)

            ## set human position in both coordinate
            self.set_human_position(d_h, theta_h, self.humans[h][0]*self.resolution, self.humans[h][1]*self.resolution)
            self.set_robot_destination(self.dests[0][0]*self.resolution, self.dests[0][1]*self.resolution)


            # c_min = 100.
            # c_max = 100.
            for i in range(bpf_array.shape[0]):
                # print ""
                for j in range(bpf_array.shape[1]):
                    if not (i == (self.humans[h][0] or self.dests[0][0]) and j == (self.humans[h][1] or self.dests[0][1])):
                        coef_att = self.get_attractive_force(i*self.resolution, j*self.resolution) * self.gamma
                        Fx, Fy, F, coef_rep = self.get_repulsive_force(i*self.resolution, j*self.resolution, h)
                        coef_rep = coef_rep*self.gamma
                        # sys.stdout.write(str(coef) + " ")
                        if coef_att > coef_rep:
                            coef = coef_att
                        else:
                            coef = coef_rep
                        # coef = max(coef_att, coef_rep)

                        if bpf_array[i][j] < coef:
                            bpf_array[i][j] = coef

                        """ 
                        ## To make cv2 image to show cv2.imshow
                        if (bpf_array_for_cv2[i][j] < coef) and coef < 255:
                            bpf_array_for_cv2[i][j] = coef
                        elif coef > 255.:
                            bpf_array_for_cv2[i][j] = 255.
                        """
                        # if c > c_max:
                        #     c_max = c
                        # if c < c_min:
                        #     c_min = c

            # print c_max, c_min

            # plt.quiver(self.humans[h][1]*self.resolution,self.humans[h][0]*self.resolution,self.human_vels[h][1],self.human_vels[h][0], angles='xy',scale_units='xy',scale=1)
            # plt.draw()


            print ""
            print "-------------  Num of humans: "+str(h+1)+" / "+str(len(self.humans))+" ------------"
            print "human position: "
            print "(x, y) = ("+str(self.x_h[h])+"[m], "+str(self.y_h[h])+"[m]) "
            print "(r, theta) = ("+str(self.d_h[h])+"[m], "+str(round(math.degrees(self.theta_h[h]),2))+"[degree])"
            print "human's velocity(direction) : "
            print "(hVx, hVy) = ("+str(self.human_vels[h][0])+", "+str(self.human_vels[h][1])+")"
            print "human direction = arctan(hVy/hVx) = "+str(round(math.degrees(math.atan2(self.human_vels[h][1], self.human_vels[h][0])),2))+"[degree])"
            print "------------------------------------------------"
            

        ## show the bpf_array(plt)
        # plt.contour(X, Y, bpf_array, 20, cmap='viridis')
        # plt.colorbar()
        # plt.show()

        ## show the bpf_array(cv2)
        # cv2.imshow('result',bpf_array_for_cv2)
        # key = cv2.waitKey(0)&0xff
        # # if key == ord('q'):
        # cv2.destroyAllWindows()

        bpf_max = bpf_array.max() 
        data = bpf_array / bpf_max.astype(float) * 100
        data = list(data.astype(int).flat)

        # print type(data)

        return data

    def get_polar_coordinate_from_origin(self, x, y):
        x = float(x)
        d = math.sqrt((x**2 + ((self.n)*self.resolution-y)**2))
        theta = math.atan2(((self.n)*self.resolution-y) , x)

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
        x_r = float(x_r)
        y_r = float(y_r)
        r = math.sqrt(((x_r - self.robot_dn_x)**2 + (y_r - self.robot_dn_y)**2))
        theta = math.atan2((y_r - self.robot_dn_x) , (x_r - self.robot_dn_x))
        return r, theta #Actually, it need to return only r

    
    def get_attractive_force(self, x_r, y_r): # 0.0m < x, y < 1.9m

        d_hr, theta_hr = self.get_distance_to_the_destination(x_r, y_r)
        # print "(d_hr, theta_hr) = ("+str(d_hr)+", "+str(math.degrees(theta_hr))+")"

        return  self.kappa_att * d_hr

    def get_repulsive_force(self, x_r, y_r, h): # 0.0m < x, y < 1.9m

        d_hr, theta_hr = self.get_relative_position(x_r, y_r, h)
        # print "(d_hr, theta_hr) = ("+str(d_hr)+", "+str(math.degrees(theta_hr))+")"
    
        mu = math.atan2(self.human_vels[h][1], self.human_vels[h][0])
        coef = self.coefficientRisk(d_hr, theta_hr, self.human_vels[h][0], self.human_vels[h][1], self.kappa, mu)
        Fx = self.vectorRiskX(d_hr, theta_hr, self.kappa, coef, mu)
        Fy = self.vectorRiskY(d_hr, theta_hr, self.kappa, coef, mu)
        F = math.sqrt(Fx**2 + Fy**2)
        # print "F = ("+str(F)+")"

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
    bP = BehaviorPotentialField()
    bP.show_bpf()


if __name__ == '__main__':
    main()

