#!/usr/bin/env python
import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt
import math
# from pyquaternion import Quaternion

from openpose_skeleton_3d.msg import COCO3d_ARR
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped

from waypoints import ComputeWaypoints
from visualize_potentials import BehaviorPotentialField
from time import sleep

import tf


class subPose3d:
	def __init__(self):
		self.width = 400 #[cells]
		self.height = 400 #[cells]
		self.resolution = 0.01 #[m/cells]
		self.start_position_x = 0 #[m] 
		self.start_position_y = 0 #[m]

		""" About the pioneer """
		self.m_pioneer = 30.0 #[kg]
		self.V0 = np.array([0.0, 0.0]) #[m/s]
		self.a = 100.

		""" These are coefficients to calculate the potential """
		self.dests = []
		self.dest1_x = 3.5 #[m]
		self.dest1_y = 1.0 #[m]
		self.dests.append([self.dest1_x,self.dest1_y])

		self.kappa = 0.5
		self.alpha = 200.0 #300.0 #20000 #700
		self.beta = 1.0
		self.sigma = 1.0 #[m] #defalt is 1.5
		self.gamma = 30. #It seems like 25. is better for now

		self.kappa_att = 2.5 # gradient of the attractive force
		# self.delta = 1.0

		self.epsilon = 180 # curvature
		self.zeta = 0.1 #[m] # Threshold of the distance from robot to the goal

		rospy.init_node('subPose3d_node', anonymous=True)
		self.pub_map = rospy.Publisher('/potential_field_ros/grid_map', OccupancyGrid, queue_size=10)

		self.pub_waypoint = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)


		## Grid_map configuration
		self.grid = OccupancyGrid()
		self.grid.header.frame_id = "world"
		self.grid.header.stamp = rospy.Time.now()
		self.grid.info.resolution = self.resolution
		self.grid.info.width = self.width
		self.grid.info.height = self.height
		self.grid.info.origin.position.x = -1 * self.width*self.resolution/2
		self.grid.info.origin.position.y = -1 * self.height*self.resolution/2

		## PoseWithCovarianceStamped configuration
		self.goals = PoseWithCovarianceStamped()
		self.goals.header.frame_id = "kinect_optical_link" #"base_link"??
		self.goals.header.stamp = rospy.Time.now()


	def get_pose_3d(self):
		# try:
		rospy.loginfo("Getting pose...")
		pose = rospy.wait_for_message("/openpose_skeleton_3d_node/pose3d",  COCO3d_ARR) #should it be set some timeout?
		# print pose

		## Calculate human orientation here
		self.define_humans_from_pose(pose)

		# self.humans = []
		# self.human_vels = []
		# self.define_humans()

		self.waypoints = ComputeWaypoints(self)
		self.pf = BehaviorPotentialField(self)

		self.show_waypoints()
	
	def show_waypoints(self):
		# self.waypoints.show_waypoints()
		# self.pf.show_bpf()

		reset_goal = True
		goal_is_human = False
		human_position = None

		while reset_goal is True:
			print "Where do you want to go ?"

			for i in range(len(self.humans)):
				print "  - human "+str(i)+" : ("+str(self.humans[i][0])+", "+str(self.humans[i][1])+")"
			
			x, y = map(float, raw_input("input goal position as (x,y)[m] ex. 3.5 1.0 >> ").split())
			print x, y
			
			# if x < self.width and y < self.height:
			for i in range (len(self.humans)):
				distance_to_the_human = math.sqrt((x-self.humans[i][0])**2+(y-self.humans[i][1])**2)
				if distance_to_the_human < 0.2:
					print "Now I'm gonna go to human no."+str(i)+": ("+str(self.humans[i][0])+", "+str(self.humans[i][1])+")"
					goal_is_human = True
					human_position = i
			
			if not goal_is_human:
				print "Now I'm gonna go to ("+str(x)+", "+str(y)+")"

			self.reset_goal_position(x, y, goal_is_human, human_position)
			self.waypoints.show_waypoints()
			self.Let_the_robot_move()

			# else:
			# 	reset_goal = False
			# 	sleep(5)

			print " --------- ------- -------- ------- ------- "
			rospy.loginfo("Now getting pose...")
			pose = rospy.wait_for_message("/openpose_skeleton_3d_node/pose3d",  COCO3d_ARR)
			self.define_humans_from_pose(pose)
			print " --------- ------- -------- ------- ------- "

		print "Done."

	
	def Let_the_robot_move(self):
		self.long_waypoints = []
		for i in range(len(self.waypoints.waypoints)):
			if i %3 == 2:
				self.long_waypoints.append(self.waypoints.waypoints[i])

		print len(self.long_waypoints)

		rate = rospy.Rate(0.5)
		# while not rospy.is_shutdown():
		for i in range(len(self.long_waypoints)-1):
			transformed_waypoint = self.transform_waypoint(i)
			next_transformed_waypoint = self.transform_waypoint(i+1)
			
			self.goals.pose.pose.position.x = transformed_waypoint.point.x
			self.goals.pose.pose.position.y = transformed_waypoint.point.y
			self.goals.pose.pose.position.z = transformed_waypoint.point.z
			
			Vx = next_transformed_waypoint.point.x - transformed_waypoint.point.x
			Vy = next_transformed_waypoint.point.y - transformed_waypoint.point.y
			Vz = 0.
			V = np.array([Vx, Vy, Vz])
			V = V / np.linalg.norm(V)

			# R = np.array([[Vx, -Vy, 0], [Vy, Vx, 0], [0.,0.,1]])
			R = np.array([[V[0], -V[1], V[2]], [V[1], V[0], 0], [V[2],0.,1]])
			# R = np.array([[Vx, Vy, 0], [-Vy, Vx, 0], [0.,0.,1]])
			M = np.eye(4, dtype=np.float64)
			M[:3, :3] = R
			# print M
			print "a"
			q = tf.transformations.quaternion_from_matrix(M)
			print "b"

			# q = Quaternion(matrix=R)
			self.goals.pose.pose.orientation.x = q[0]
			self.goals.pose.pose.orientation.y = q[1]
			self.goals.pose.pose.orientation.z = q[2]
			self.goals.pose.pose.orientation.w = q[3]

			self.pub_waypoint.publish(self.goals)
			print self.goals

			rate.sleep()
    
	# def kinect_to_odom(self):
	# 	listener = tf.TransformListener()
	# 	try:
	# 		listener.transformPoint("/odom", "/kinect_optical_frame", waypoint, transformed_waypoint)

	def transform_waypoint(self, i):
		waypoint = PointStamped()
		waypoint.header.frame_id = "kinect_optical_link"
		waypoint.header.stamp = rospy.Time(0)
		# waypoint.point.x = self.waypoints.waypoints[i][1]
		waypoint.point.x = self.long_waypoints[i][1]
		waypoint.point.y = 0.0
		# waypoint.point.z = self.waypoints.waypoints[i][0]
		waypoint.point.z = self.long_waypoints[i][0]

		listener = tf.TransformListener()
		# listener.waitForTransform("/kinect_optical_link", "/odom", rospy.Time(0), rospy.Duration(4.0))
		listener.waitForTransform("/odom", "/kinect_optical_link", rospy.Time(0), rospy.Duration(4.0))
		transformed_waypoint = listener.transformPoint("odom", waypoint)

		return transformed_waypoint


	def reset_goal_position(self, x, y, goal_is_human, human_position):
		# reset new goal
		self.dests = []
		self.dests.append([x, y])
		self.waypoints.dests = self.dests
		last_point = self.waypoints.waypoints[-1]
		self.waypoints.waypoints = []
		self.waypoints.waypoints.append(last_point)
		self.pf.dests = self.dests

		## check if goal is human or not
		# np_humans = np.array(self.humans)
		# xy = np.array([x,y])
		# goal_is_human = False
		# for i in range(len(np_humans)):
		# 	if (np_humans[i] == xy).all() == True:
		# 		print np_humans[i]
		# 		goal_is_human = True
		# 		human_position = i

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
		
	
	def define_humans_from_pose(self, pose):
		self.humans = []
		self.human_vels = []
		print "--------------- Map meta data -----------------"
		print "num of human : "+str(len(pose.data))
		
		for i in range(len(pose.data)):
			# Human position
			if pose.data[i].Neck.z != 0.0:
				self.humans.append([round(pose.data[i].Neck.z,2), round(pose.data[i].Neck.x,2)])

			elif pose.data[i].RShoulder.z != 0.0 and pose.data[i].LShoulder.z != 0.0:
				x = (pose.data[i].RShoulder.z - pose.data[i].LShoulder.z)/2. + pose.data[i].LShoulder.z
				y = (pose.data[i].RShoulder.x - pose.data[i].LShoulder.x)/2. + pose.data[i].LShoulder.x 
				# y = (pose.data[i].RShoulder.x - pose.data[i].LShoulder.x)/2. + pose.data[i].LShoulder.x + self.width/2*self.resolution
				self.humans.append([round(x,2), round(y,2)])

			print "human "+str(i)+" = ("+str(self.humans[i][0])+", "+str(self.humans[i][1])+")"

			# Human Orientation
			if pose.data[i].RShoulder.z != 0.0 and pose.data[i].LShoulder.z != 0.0:
				x = pose.data[i].RShoulder.z - pose.data[i].LShoulder.z
				y = pose.data[i].RShoulder.x - pose.data[i].LShoulder.x

				normal_1 = [-y, x]
				normal_2 = [y, -x]
				if pose.data[i].RShoulder.z < pose.data[i].LShoulder.z:
					if normal_1[0] > 0:
						LR_normal = np.array(normal_1)
				   		LR_normal = LR_normal / np.linalg.norm(LR_normal)
						self.human_vels.append(LR_normal)
					else:
						LR_normal = np.array(normal_2)
				   		LR_normal = LR_normal / np.linalg.norm(LR_normal)
						self.human_vels.append(LR_normal)
				else:
					if normal_1[0] < 0:
						LR_normal = np.array(normal_1)
				   		LR_normal = LR_normal / np.linalg.norm(LR_normal)
						self.human_vels.append(LR_normal)
					else:
						LR_normal = np.array(normal_2)
				   		LR_normal = LR_normal / np.linalg.norm(LR_normal)
						self.human_vels.append(LR_normal)
				
				print "RShoulder "+str(i)+" = ("+str(pose.data[i].RShoulder.z)+", "+str(pose.data[i].RShoulder.x)+")"
				print "LShoulder "+str(i)+" = ("+str(pose.data[i].LShoulder.z)+", "+str(pose.data[i].LShoulder.x)+")"
				print "human orientation "+str(i)+" = ("+str(self.human_vels[i][0])+", "+str(self.human_vels[i][1])+")"


				# plt.quiver(self.humans[i][1], self.humans[i][0], self.human_vels[i][0], self.human_vels[i][1], color='g', angles='xy',scale_units='xy',scale=1)
				# plt.draw()
			
			else:
				LR_normal = [0. ,0.]
				self.human_vels.append(LR_normal)
				print "human orientation"+str(i)+" = ("+str(self.human_vels[i][0])+", "+str(self.human_vels[i][1])+")"
				# plt.scatter(self.humans[i][1], self.humans[i][0], marker='o', color='b') 
			
			print ""
			self.num_humans = len(self.humans)



	def define_humans(self):
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


def main(args):
	pose = subPose3d()
	# rospy.init_node('subPose3d_node', anonymous=True)
	try:
		# rospy.Subscriber('/openpose_skeleton_3d_node/pose3d', COCO3d_ARR, pose.callback)
		pose.get_pose_3d()
		rospy.spin() #?
	except KeyboardInterrupt:
		print "Shutting down ROS module"
		del hu

if __name__=='__main__':
	main(sys.argv)
	