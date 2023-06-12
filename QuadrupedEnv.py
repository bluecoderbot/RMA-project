import gym
from gym.spaces import Box
import numpy as np

import copy
from QuadrupedSim import QuadrupedSim
import os
#os.environ['PYBULLET_USE_CUDA'] = '1'

# Suppose each joint has an angle between -pi and pi and a velocity between -10 and 10
joint_low_bounds = [-4] * 12
joint_high_bounds = [4] * 12

joint_low_velocity = [-8] * 12
joint_high_velocity = [8] * 12

roll_low = [-3.15]  # -180 degrees in radians
roll_high = [3.15]  # 180 degrees in radians

pitch_low = [-3.15]  # -180 degrees in radians
pitch_high = [3.15]  # 180 degrees in radians



contact_indicators_low = [0]*4 #binarized foot contact indicators
contact_indicators_high = [1]*4 #binarized foot contact indicators


TARGET_LIMITS = 50
TARGET_THRESHOLD = 2


# Suppose each joint can be commanded to go to an angle between -pi and pi
action_low_bounds = [-np.pi] * 12
action_high_bounds = [np.pi] * 12

low_bounds=joint_low_bounds+joint_low_velocity+roll_low+pitch_low+contact_indicators_low + action_low_bounds
high_bounds=joint_high_bounds+joint_high_velocity+roll_high+pitch_high+contact_indicators_high + action_high_bounds


#environment variables
lowmass = 13
highmass = 20

lowmotor = 10
highmotor = 20

lowFriction = 0.04
highFriction = 6

lowfootheight = 0
highfootheight = 1.2


low_environment = [lowmass] + [lowmotor]*12 + [lowFriction] + [lowfootheight]
high_environment = [highmass] + [highmotor]*12 + [highFriction] + [highfootheight]


toenamekeys = ['toeRL', 'toeRR', 'toeFL', 'toeFR']

rewardNames = ['Forward', 'Lateral Movement and Rotation', 'Work', 'Ground Impact', 'Smoothness', 'Action Magnitude', 'Joint Speed', 'Orientation', 'Z Acceleration', 'Foot Slip']

class QuadrupedEnv(gym.Env):


	def __init__(self, simfrequency = 400, agentFrequency=100, max_episode_len = 1000, min_height = 0.42, reward_scaling = [20,21,0.002,0.02,0.001, 0.07,0.002,1.5,2.0,0.8], curriculum_base = 0.03, curriculum_power = 0.997, include_env_vars = False, include_adapter_vars = False, full_batch_size = 20000, mini_batch_size=5000, epochs= 10):
		super().__init__()

		if include_env_vars and include_adapter_vars:
			raise ValueError("You can't run phase 1 and 2 at the same time")

		if include_env_vars:
			self.observation_space = Box(low=np.array(low_bounds + low_environment), high=np.array(high_bounds + high_environment))
		else:
			self.observation_space = Box(low=np.array(low_bounds), high=np.array(high_bounds))

		self.action_space = Box(low=np.array(action_low_bounds), high=np.array(action_high_bounds))

		self.epochs = epochs

		self.full_batch_size = full_batch_size
		self.mini_batch_size = mini_batch_size

		self.current_reward_penalties = np.array([0]*9) 

		self.maxsteps = max_episode_len

		self.simfrequency = simfrequency
		self.agentFrequency = agentFrequency

		self.min_height = min_height#0.28*0.6/0.4 = 0.42  | 0.28 is maliks threshold, 0.6 is the height of laikago, 0.4 is the height of A1

		self.num_steps = 0
		self.total_num_steps = 0

		self.reward_scaling = reward_scaling
		self.curriculum_factor = curriculum_base
		self.curriculum_power = curriculum_power  

		#previous iteration
		self.previous_angles = np.zeros([0]*12)
		self.previous_ground_reaction_force = np.zeros((4, 3))
		self.previous_torques = np.array([0]*12)
		self.previous_total_velocity = np.array([0,0,0])
		self.previous_action = [0]*12

		self.sim = QuadrupedSim(tstep = 1./simfrequency)



	def reward(self, stateVariables = [], rewardvariables = []):

		reward = 0
		rewardterms = []
		##############################################################################################################
		vx = rewardvariables[0]

		ForwardgoalReward = min(vx, 0.35)
		reward += self.reward_scaling[0]*ForwardgoalReward
		##############################################################################################################
		vy = rewardvariables[1]
		wy = rewardvariables[2]

		lateral_Movement_and_Rotation = -1*np.square(vy) - np.square(wy)

		rewardterms.append(lateral_Movement_and_Rotation)
		##############################################################################################################
		torques = np.array(rewardvariables[3])
		angles = np.array(stateVariables[0:12])*np.array(self.sim.jointDirections) + np.array(self.sim.jointOffsets)

		deltavelocity = angles - self.previous_angles


		jointvelocity = np.array(stateVariables[12:24])*np.array(self.sim.jointDirections)


		delta_positve_velocity_negative = (deltavelocity>0)*(jointvelocity<0) * (-2*np.pi)

		delta_negative_velocity_positve = (deltavelocity<0)*(jointvelocity>0) * 2*np.pi

		deltavelocity = deltavelocity + delta_positve_velocity_negative + delta_negative_velocity_positve


		if np.sum(1*(deltavelocity<-2*np.pi)+ 1*(deltavelocity>2*np.pi))  >0:
			print('mistake detected in delta calculations')

		#if self.total_num_steps%100 == 0:
			#print('angles: ', angles[0], '\nself.previous_angles :', self.previous_angles[0], '\nangle difference : ', angles[0]- self.previous_angles[0], '\nangular velocity: ', stateVariables[12]) 
			#print('\n\n')


		Work = -1*np.abs( np.sum(torques*deltavelocity ) )

		self.previous_angles = angles.copy()

		rewardterms.append(Work)
		##############################################################################################################
		ground_reaction_force = rewardvariables[4]

		GroundImpact = -1*( np.linalg.norm(ground_reaction_force - self.previous_ground_reaction_force) )**2

		#if self.total_num_steps%10 == 0:
			#print("ground_reaction_force : ", ground_reaction_force.shape, " previous_ground_reaction_force: " , self.previous_ground_reaction_force.shape)

		rewardterms.append(GroundImpact)
		##############################################################################################################

		Smoothness = -1* ( np.linalg.norm(torques - self.previous_torques) )**2

		self.previous_torques = torques.copy()

		rewardterms.append(Smoothness)
		##############################################################################################################

		vz = rewardvariables[5]
		total_velocity = np.array([vx, vy, vz])
		AccelerationMagnitude = -1* ( np.linalg.norm(total_velocity - self.previous_total_velocity) )**2 

		self.previous_total_velocity = total_velocity.copy()

		rewardterms.append(AccelerationMagnitude)
		##############################################################################################################

		JointSpeed = -1* ( np.linalg.norm(jointvelocity) )**2 

		rewardterms.append(JointSpeed)
		##############################################################################################################

		roll = stateVariables[24]
		pitch = stateVariables[25] - np.pi/2
		Orientation = -1* ( np.linalg.norm( np.array([roll, pitch]) ) )**2 

		rewardterms.append(Orientation)

		##############################################################################################################
	

		Zacceleration = -1* (vz)**2

		rewardterms.append(Zacceleration)


		##############################################################################################################
		foot_contact = np.array( stateVariables[26:30])
		foot_velocity = rewardvariables[6]

		FootSlip = -1* (    np.linalg.norm(  np.matmul(np.diag(foot_contact),foot_velocity)  )    )**2


		#if self.total_num_steps%10 == 0:
			#print("diag : ", np.diag(foot_contact).shape, " foot_velocity shape: " , foot_velocity.shape)
		rewardterms.append(FootSlip)
		##############################################################################################################

		for indx in range(len(rewardterms)):

			rterm = rewardterms[indx]*self.reward_scaling[indx+1]

			if indx>0:
				rterm=rterm*self.curriculum_factor
			if rterm>0:
				print("reward penalty term greater than 0: ", rterm, " index ", indx)


			#if self.total_num_steps%10 == 0:
				#print("rewardterms ", rewardNames[indx+1],": ", rewardterms[indx])


			self.current_reward_penalties[indx] = rterm

			reward+= rterm

		#print("\n")

		return reward

	def step(self, action):

		#print('step: ', self.num_steps)


		self.sim.set_actions(action)




		for s in range(int(self.simfrequency/self.agentFrequency)):
			self.sim.stepSim()


		obs = self.sim.get_object_state()
		observation = obs[0]
		rewardvariables = obs[1]
		encodervariables = obs[2]
		resetvariables = obs[3]

		height = resetvariables[0]
		roll = observation[24]
		pitch = observation[25] - np.pi/2

		reward = self.reward(observation, rewardvariables)





		self.num_steps +=1
		self.total_num_steps +=1

		if self.total_num_steps%self.full_batch_size == 0:
			for s in range(int(self.epochs*(self.full_batch_size/self.mini_batch_size) ) ):
				self.curriculum_factor = np.power(self.curriculum_factor, self.curriculum_power)
				print('New curriculum_factor (', s, '): ', self.curriculum_factor)



		done = False
		#if self.num_steps>=self.maxsteps or height<self.min_height or roll<-0.4 or roll>0.4 or pitch<-0.2 or pitch>0.2:
		if self.num_steps>=self.maxsteps or height<self.min_height or roll<-0.4*1.5 or roll>0.4*1.5 or pitch<-0.2*1.5 or pitch>0.2*1.5:

			#if self.num_steps>=self.maxsteps:
			#	print("self.num_steps>=self.maxsteps")

			#elif height<self.min_height:
			#	print("height<self.min_height ,", height)

			#elif roll<-0.8:
			#	print("roll<-0.8, ", roll)

			#elif roll>0.8:
			#	print("roll>0.8, ", roll)

			#elif pitch<-0.4:
			#	print("pitch<-0.4", pitch)

			#elif pitch>0.4:
			#	print("pitch>0.4", pitch)

			done = True

		info = {}


		observationFinal = observation + self.previous_action


		self.previous_action = np.array(action).tolist()

		return observationFinal, reward, done, info


	def getCurriculum_factor(self):
		return self.curriculum_factor


	def getcurrent_reward_penalties(self):
		return self.current_reward_penalties

	def reset(self):

		self.num_steps=0


		#previous iteration
		self.previous_angles = [0]*12
		self.previous_ground_reaction_force = np.zeros((4, 3))
		self.previous_torques = [0]*12
		self.previous_total_velocity = np.array([0,0,0])


		self.sim.reset_sim()


		# Compute and return the initial observation
		initial_observation = self.sim.get_object_state()[0]

		self.previous_action = (initial_observation[0:12]).copy() 

		initial_observation2 = initial_observation  + self.previous_action.copy() 




		# return observation
		return initial_observation2




#qe = QuadrupedEnv()


