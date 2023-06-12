import torch.nn as nn

class EnvironmentFactorEncoder(nn.Module):
    def __init__(self, observation_space, processed_dim=8):
        super(EnvironmentFactorEncoder, self).__init__()

        n_input = 15  # The first 15 values of observation
        self.other_dim = observation_space.shape[0] - n_input

        # Define 3 layer MLP
        self.net = nn.Sequential(
            nn.Linear(n_input, 256),
            nn.ReLU(),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Linear(128, processed_dim),
            nn.ReLU(),
        )

    def forward(self, observations):
        processed_observations = self.net(observations[:, :15])
        other_observations = observations[:, 15:]
        return torch.cat((processed_observations, other_observations), dim=1)


from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy


class CustomActorCriticPolicy(ActorCriticPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomActorCriticPolicy, self).__init__(*args, **kwargs,
                                                      net_arch=[128, dict(pi=[128], vf=[128])],
                                                      features_extractor_class=EnvironmentFactorEncoder,
                                                      features_extractor_kwargs=dict(processed_dim=8))

