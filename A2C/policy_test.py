import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from models import MlpPolicy
from noVis_sim import DroneEnv


def load_models(state_size, n_drones, action_size):
    nets = []
    for i in range(n_drones):
        model = MlpPolicy(state_size, n_drones, action_size)
        model.load_state_dict(torch.load(f"A2C_models/A2C_drone_{i}.bin"))
        nets.append(model)
    return nets


env = DroneEnv(row_count=5, col_count=5, step_size=1.0)

obs_size = env.state_size
action_size = env.action_size
n_drones = env.n_drones

nets = load_models(obs_size, n_drones, action_size)

obs = env.reset()
obs = np.reshape(obs, [1, obs_size])
drone_pos = np.array(env.n_drones_pos)
drone_pos = drone_pos.reshape((n_drones, 2))
done = False

step = 0
print(f"Step: {step+1}")
print(f"Drone positions:{env.n_drones_pos}")
temp_st = obs.reshape((5, 5))
print(temp_st.T, "\n")

while not done:
    obs = torch.from_numpy(obs).float()
    drone_pos = torch.from_numpy(drone_pos).float().view(1, -1)

    policies = []
    values = []
    actions = []
    for i in range(n_drones):
        p, v = nets[i](obs, drone_pos)
        probs = F.softmax(p, dim=-1)
        a = probs.multinomial(probs.size()[0]).data

        policies.append(p)
        values.append(v)
        actions.append(a.numpy()[0, 0])

    obs, reward, done = env.step(actions)
    obs = np.reshape(obs, [1, obs_size])
    drone_pos = np.array(env.n_drones_pos)
    drone_pos = drone_pos.reshape((n_drones, 2))

    if (step + 1) % 100 == 0:
        print(f"Step: {step+1}")
        print(f"Drone positions:{env.n_drones_pos}")
        print(f"Action:{actions}")
        temp_st = obs.reshape((5, 5))
        print(temp_st.T, "\n")

    step += 1
