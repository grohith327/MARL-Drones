import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable


def ortho_weights(shape, scale=1.0):
    """ PyTorch port of ortho_init from baselines.a2c.utils """
    shape = tuple(shape)

    if len(shape) == 2:
        flat_shape = shape[1], shape[0]
    elif len(shape) == 4:
        flat_shape = (np.prod(shape[1:]), shape[0])
    else:
        raise NotImplementedError

    a = np.random.normal(0.0, 1.0, flat_shape)
    u, _, v = np.linalg.svd(a, full_matrices=False)
    q = u if u.shape == flat_shape else v
    q = q.transpose().copy().reshape(shape)

    if len(shape) == 2:
        return torch.from_numpy((scale * q).astype(np.float32))
    if len(shape) == 4:
        return torch.from_numpy(
            (scale * q[:, : shape[1], : shape[2]]).astype(np.float32)
        )


class Policy(nn.Module):
    def __init__(self, obs_size, n_drones, action_size, policy_type="MLP"):
        super(Policy, self).__init__()
        if policy_type == "MLP":
            self.actor = MlpPolicy(obs_size, n_drones, action_size)
            self.critic = MlpPolicy(obs_size, n_drones, 1)
        else:
            self.actor = CNNPolicy(n_drones, action_size)
            self.critic = CNNPolicy(n_drones, 1)

    def forward(self, state, drone_pos):
        pi = self.actor(state, drone_pos)
        v = self.critic(state, drone_pos)
        return pi, v


class MlpPolicy(nn.Module):
    def __init__(self, obs_size, n_drones, action_size):
        super(MlpPolicy, self).__init__()
        self.state_input = nn.Linear(obs_size, 128)
        self.drone_input = nn.Linear(n_drones * 2, 128)
        self.dense1 = nn.Linear(256, 128)
        self.dense2 = nn.Linear(128, 64)
        self.dense3 = nn.Linear(64, 32)
        self.dense4 = nn.Linear(32, action_size)

        self._init_weights()

    def _init_weights(self):
        layers = [
            self.state_input,
            self.drone_input,
            self.dense1,
            self.dense2,
            self.dense3,
            self.dense4,
        ]
        for layer in layers:
            layer.weight.data = ortho_weights(layer.weight.size())

    def forward(self, state, drone_pos):
        mu = state.mean()
        std = state.std()
        state = (state - mu) / std
        state_embed = F.tanh(self.state_input(state))
        drone_embed = F.tanh(self.drone_input(drone_pos))
        out = torch.cat([state_embed, drone_embed], dim=-1)
        out = F.normalize(out)
        out = F.tanh(self.dense1(out))
        out = F.tanh(self.dense2(out))
        out = F.tanh(self.dense3(out))
        out = self.dense4(out)
        return out


class CNNPolicy(nn.Module):

    """
    Policy for 5x5 grid
    """

    def __init__(self, n_drones, action_size):
        super(CNNPolicy, self).__init__()
        self.conv1 = nn.Conv2d(1, 16, 2)
        self.drone_input = nn.Linear(n_drones * 2, 128)
        self.conv2 = nn.Conv2d(16, 32, 2)
        self.conv3 = nn.Conv2d(32, 64, 2)
        self.conv4 = nn.Conv2d(64, 128, 2)
        self.dense1 = nn.Linear(256, 128)
        self.dense2 = nn.Linear(128, 64)
        self.dense3 = nn.Linear(64, 32)
        self.dense4 = nn.Linear(32, action_size)

        self.batch_norm = nn.BatchNorm2d(32)

        self._init_weights()

    def _init_weights(self):
        layers = [
            self.drone_input,
            self.dense1,
            self.dense2,
            self.dense3,
            self.dense4,
        ]
        for layer in layers:
            layer.weight.data = ortho_weights(layer.weight.size())

    def forward(self, state, drone):
        ## Normalize
        mu = state.mean()
        std = state.std()
        state = (state - mu) / std
        state = state.unsqueeze(1)
        ## Conv operations
        state = F.tanh(self.conv1(state))
        state = F.tanh(self.conv2(state))
        state = self.batch_norm(state)
        state = F.tanh(self.conv3(state))
        state = F.tanh(self.conv4(state))
        ## Concat
        state = state.view(-1, 128)
        drone = F.tanh(self.drone_input(drone))
        out = torch.cat([state, drone], dim=-1)
        out = F.normalize(out)
        ## Predict policies and values
        out = F.tanh(self.dense1(out))
        out = F.tanh(self.dense2(out))
        out = F.tanh(self.dense3(out))
        out = self.dense4(out)
        return out
