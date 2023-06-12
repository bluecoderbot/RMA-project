from gym import Env
from gym_code import QuadrupedEnv


env = QuadrupedEnv()



episodes = 200
for episode in range(1, episodes+1):
	state = env.reset()
	done = False
	score = 0


	#while not done:
	for i in range(5):
		action = env.action_space.sample()
		print(action)
		observation, reward, done, info = env.step(action)
		score += reward
	print('EPISODE: ', episode, ', score: ', score )


import stable_baselines3
from stable_baselines3.sac.policies import MlpPolicy
from stable_baselines3 import SAC


model = SAC(MlpPolicy, env, verbose=1, device="cpu", tensorboard_log = "./a2c_cartpole_tensorboard/")
model.learn(total_timesteps = 5000000, log_interval = 10)


print('created model')
