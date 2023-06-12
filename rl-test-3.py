from gym import Env
from QuadrupedEnv import QuadrupedEnv
import time
import torch as th

FULL_BATCHSIZE = 80000
MINI_BATCHSIZE = 20000
CURRICULUM_BASE = 0.03
NUM_EPOCHS = 10

env = QuadrupedEnv(full_batch_size = FULL_BATCHSIZE, curriculum_base = CURRICULUM_BASE, mini_batch_size = MINI_BATCHSIZE, epochs = NUM_EPOCHS)

state = env.reset()

for i in range(1000):
	env.sim.stepSim()
obs = env.sim.get_object_state()

#time.sleep(30)

cttr = 0
while(False):
	env.sim.stepSim()
	obs = env.sim.get_object_state()

	cttr+=1


episodes = 0
for episode in range(1, episodes+1):
	state = env.reset()
	done = False
	score = 0


	#while not done:
	for i in range(5):
		action = env.action_space.sample()
		print('action', len(action) )
		observation, reward, done, info = env.step(action)
		print('observation:', len(observation))
		score += reward
	print('EPISODE: ', episode, ', score: ', score )


env.total_num_steps = 0

from stable_baselines3.common.callbacks import BaseCallback



class CustomCallback(BaseCallback):
    def _on_step(self):
        value = self.training_env.envs[0].getCurriculum_factor()

        penalties = self.training_env.envs[0].getcurrent_reward_penalties()

        self.logger.record('Custom_Reward/curriculum_factor', value)


        for i in range(len(penalties)):
                self.logger.record('Custom_Reward/term_'+str(i), penalties[i])
        return True


custom_callback = CustomCallback()

from ppo import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

savepath = "flat_velocity_MLP_noadapter-2.zip"

env = DummyVecEnv([lambda: env])
policy_kwargs = dict(
    activation_fn=th.nn.ReLU, 
    net_arch=[128, dict(vf=[128], pi=[128])],
    optimizer_class=th.optim.Adam,
    optimizer_kwargs=dict( eps=1e-8, betas=(0.9, 0.999))
)

model = PPO('MlpPolicy', env, verbose = 1, device="cuda", learning_rate = 0.0005, policy_kwargs=policy_kwargs, batch_size = MINI_BATCHSIZE, vf_coef = 0.5, clip_range = 0.2, gae_lambda=0.95, gamma=0.998, n_steps = FULL_BATCHSIZE, tensorboard_log = "./ppo_laikago_flat_low_pitchRoll_reset_MLP_noadapter_StdMin0-2_tensorboard/")

print('created model')

model.learn(total_timesteps = 10000000, callback=custom_callback)
model.save(savepath)

model.learn(total_timesteps = 10000000, callback=custom_callback)
model.save(savepath)

model.learn(total_timesteps = 10000000, callback=custom_callback)
model.save(savepath)
