#!/usr/bin/env python
import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt

from openpose_skeleton_3d.msg import COCO3d_ARR
from nav_msgs.msg import OccupancyGrid

from waypoints import ComputeWaypoints
from visualize_potentials import BehaviorPotentialField
from time import sleep


class subPose3d:
	def __init__(self):
		self.width = 400 #[cells]
		self.height = 400 #[cells]
		self.resolution = 0.01 #[m/cells]
		self.start_position_x = 299 # int(random.uniform(0, self.width)) # 240
		self.start_position_y = 200 # int(random.uniform(0, self.width)) # 240

		""" About the pioneer """
        self.m_pioneer = 10.0 #[kg]
        self.V0 = np.array([0.0, 0.0]) #[m/s]
        self.a = 300.


		""" These are coefficients to calculate the potential """
		self.dests = []
		self.dest1_x = 10
		self.dest1_y = 290
		self.dests.append([self.dest1_x,self.dest1_y])

		self.kappa = 0.5
		self.alpha = 200.0 #300.0 #20000 #700
		self.beta = 1.0
		self.sigma = 1.0 #[m] #defalt is 1.5
		self.gamma = 30. #It seems like 25. is better for now

		self.kappa_att = 2.5 # gradient of the attractive force
		# self.delta = 1.0

		self.epsilon = 0.2 # amount of movement
		self.zeta = 0.1 #[m] # Threshold of the distance from robot to the goal

		
		rospy.init_node('subPose3d_node', anonymous=True)
		self.pub = rospy.Publisher('/potential_field_ros/grid_map', OccupancyGrid, queue_size=10)

		self.grid = OccupancyGrid()
		self.grid.header.frame_id = "world"
		self.grid.header.stamp = rospy.Time.now()
		self.grid.info.resolution = self.resolution
		self.grid.info.width = self.width
		self.grid.info.height = self.height
		self.grid.info.origin.position.x = -1 * self.width*self.resolution/2
		self.grid.info.origin.position.y = -1 * self.height*self.resolution/2


	def callback(self, pose3d):

		# rate = rospy.Rate(0.1) #Number of publishing in 1 seconds
		#print pose3d
		# self.grid = OccupancyGrid()
		# self.grid.header.frame_id = "world"
		# self.grid.header.stamp = rospy.Time.now()
		# self.grid.info.resolution = self.resolution
		# self.grid.info.width = self.width
		# self.grid.info.height = self.height
		# self.grid.info.origin.position.x = -1 * self.width*self.resolution/2
		# self.grid.info.origin.position.y = -1 * self.height*self.resolution/2
			
		#### Compute humans position and direction here ####
		self.define_humans()
		#### now it is predefined.
		#### it should be computed by the pose3d.


		self.waypoints = ComputeWaypoints(self)
		self.pf = BehaviorPotentialField(self)
		# bp = BehaviorPotentialField(self.grid)

		# self.grid.data = bp.show_bpf() ## Whichever you want

		# while not rospy.is_shutdown():
		self.show_waypoints()
		rate.sleep()
		sleep(10)

		# for i in range(50):
		# 	for j in range(50):
		# 		self.grid.data.append(i+j)
		# self.pub.publish(self.grid)
		# print self.grid.data
	

	def get_pose_3d(self):
		# try:
		rospy.loginfo("Getting pose...")
		pose = rospy.wait_for_message("/openpose_skeleton_3d_node/pose3d",  COCO3d_ARR) #should it be set some timeout?
		# print pose

		## Calculate human orientation here
		self.define_humans_from_pose(pose)

		self.humans = []
		self.human_vels = []
		self.define_humans()

		self.waypoints = ComputeWaypoints(self)
		self.pf = BehaviorPotentialField(self)

		self.show_waypoints()
	
		# except:

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
		# self.pf.show_bpf()

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
    
	def show_only_potential(self):
		self.pf.show_bpf()
	
	def define_humans_from_pose(self, pose):
		self.humans = []
		self.human_vels = []

		for i in range(len(pose.data)):
			# Human position
			if pose.data.Neck.z != 0.0:
				self.humans.append([pose.data[i].Neck.z, pose.data[i].Neck.x])

			elif pose.data[i].RShoulder.z != 0.0 and pose.data[i].LShoulder.z != 0.0:
				x = (pose.data[i].RShoulder.z - pose.data[i].LShoulder.z)/2. + pose.data[i].LShoulder.z
				y = (pose.data[i].RShoulder.x - pose.data[i].LShoulder.x)/2. + pose.data[i].LShoulder.x
				self.humans.append([x, y])
				print "human "+str(i)+" = ("+str(self.humans[i][0])+", "+str(self.humans[i][1])+")"
			
			# Human Orientation
			if pose.data[i].RShoulder.z != 0.0 and pose.data[i].LShoulder.z != 0.0:
				x = pose.data[i].RShoulder.z - pose.data[i].LShoulder.z
				y = pose.data[i].RShoulder.x - pose.data[i].LShoulder.x
				LR = [x, y]
				LR_normal = [-y, x]
				self.human_vels.append(LR_normal)
				print "human orientation"+str(i)+" = ("+str(self.human_vels[i][0])+", "+str(self.human_vels[i][1])+")"

				plt.quiver(self.humans[i][1], self.humans[i][0], self.human_vels[i][0], self.human_vels[i][1], color='g', angles='xy',scale_units='xy',scale=1)
            	plt.draw()

			else:
				LR_normal = [0. ,0.]
				self.human_vels.append(LR_normal)
				print "human orientation"+str(i)+" = ("+str(self.human_vels[i][0])+", "+str(self.human_vels[i][1])+")"
        		plt.scatter(self.humans[i][1], self.humans[i][0], marker='o', color='b') 



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
	

#     def set_body_parts(self, data):
# 	body_parts = BodyPartElm()
# 	body_parts.part_id = -1
# 	body_parts.x = -1
# 	body_parts.y = -1
# 	body_parts.confidence = -1
#         np_body_part_id = np.full(19, -1)
# 	np_body_parts = np.full(19, body_parts)
	
#         for i in range(len(data)):
#             np_body_part_id[data[i].part_id] = data[i].part_id
#             np_body_parts[data[i].part_id] = data[i]
# 	    print "----"
# 	    print data[i].part_id
#             print "----"

#         return np_body_part_id, np_body_parts

#     def publishDetectionPose(self, persons, pose):
#             coco_arr = COCO_ARR()
#             coco_arr.header = pose.header
#             coco_arr.num_person = persons.size
# 	    print "person.size: "+str(persons.size)
#             for i in range(persons.size):
# 		print "i: "+str(i)
#             	coco = COCO()
#                 body_list, data = self.set_body_parts(persons[i].body_part)
# 		for j in range(len(data)):
# 			data[j].x *= 1920
# 			data[j].y *= 1080

# 		print data[1]
# 		print body_list
#                 if body_list[0] != -1:
#                     coco.Nose.x = data[0].x
#                     coco.Nose.y = data[0].y
#                     coco.Nose.confidence = data[0].confidence
# 		    print "data[0]: "+str(data[0])
#                 if body_list[1] != -1:
#                     coco.Neck.x = data[1].x
#                     coco.Neck.y = data[1].y
#                     coco.Neck.confidence = data[1].confidence
            
#                 if body_list[2] != -1:
#                     coco.RShoulder.x = data[2].x
#                     coco.RShoulder.y = data[2].y
#                     coco.RShoulder.confidence = data[2].confidence

#                 if body_list[3] != -1:
# 	                coco.RElbow.x = data[3].x
# 	                coco.RElbow.y = data[3].y
# 	                coco.RElbow.confidence = data[3].confidence


# 	        if body_list[4] != -1:
# 	                coco.RWrist.x = data[4].x
# 	                coco.RWrist.y = data[4].y
# 	                coco.RWrist.confidence = data[4].confidence

# 	        if body_list[5] != -1:
# 	                coco.LShoulder.x = data[5].x
# 	                coco.LShoulder.y = data[5].y
# 	                coco.LShoulder.confidence = data[4].confidence
	            
# 	        if body_list[6] != -1:
# 	                coco.LElbow.x = data[6].x
# 	                coco.LElbow.y = data[6].y
# 	                coco.LElbow.confidence = data[6].confidence
	            
# 	        if body_list[7] != -1:
# 	                coco.LWrist.x = data[7].x
# 	                coco.LWrist.y = data[7].y
# 	                coco.LWrist.confidence = data[7].confidence
	            
# 	        if body_list[8] != -1:
# 	                coco.RHip.x = data[8].x
# 	                coco.RHip.y = data[8].x
# 	                coco.RHip.confidence = data[8].confidence
	            
# 	        if body_list[9] != -1:
# 	                coco.RKnee.x = data[9].x
# 	                coco.RKnee.y = data[9].y
# 	                coco.RKnee.confidence = data[9].confidence
	            
# 	        if body_list[10] != -1:
# 	                coco.RAnkle.x = data[10].x
# 	                coco.RAnkle.y = data[10].y
# 	                coco.RAnkle.confidence = data[10].confidence
	            
# 	        if body_list[11] != -1:
# 	                coco.LHip.x = data[11].x
# 	                coco.LHip.y = data[11].y
# 	                coco.LHip.confidence = data[11].confidence
	            
# 	        if body_list[12] != -1:
# 	                coco.LKnee.x = data[12].x
# 	                coco.LKnee.y = data[12].y
# 	                coco.LKnee.confidence = data[12].confidence
	            
# 	        if body_list[13] != -1:
# 	                coco.LAnkle.x = data[13].x
# 	                coco.LAnkle.y = data[13].y
# 	                coco.LAnkle.confidence = data[13].confidence
	            
# 	        if body_list[14] != -1:
# 	                coco.REye.x = data[14].x
# 	                coco.REye.y = data[14].y
# 	                coco.REye.confidence = data[14].confidence
	            
# 	        if body_list[15] != -1:
# 	                coco.LEye.x = data[15].x
# 	                coco.LEye.y = data[15].y
# 	                coco.LEye.confidence = data[15].confidence
	            
# 	        if body_list[16] != -1:
# 	                coco.REar.x = data[16].x
# 	                coco.REar.y = data[16].y
# 	                coco.REar.confidence = data[16].confidence
	            
# 	        if body_list[17] != -1:
# 	                coco.LEar.x = data[17].x
# 	                coco.LEar.y = data[17].y
# 	                coco.LEar.confidence = data[17].confidence
	            
# 	        if body_list[18] != -1:
# 	                coco.Waist.x = data[18].x
# 	                coco.Waist.y = data[18].y
# 	                coco.Waist.confidence = data[18].confidence
# 	        coco_arr.data.append(coco)
# 		print coco_arr

#             self.pub.publish(coco_arr)
        

#         # coco_arr.data.emplace_back(coco)