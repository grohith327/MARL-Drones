import torch
import numpy as np
import argparse
from noVis_sim import DroneEnv
import sys

sys.path.insert(0, "pytorch-a2c-ppo-acktr-gail/")
from a2c_ppo_acktr.model import Policy

parser = argparse.ArgumentParser(description="A2C Evaluation")
parser.add_argument("--model_path", type=str, default="", help="model path")
parser.add_argument("--grid_size", type=int, default=5, help="Grid size, Default: 5x5")
parser.add_argument("--n_drones", type=int, default=3, help="number of drones")
parser.add_argument(
    "--n_anamolous",
    type=int,
    default=5,
    help="number of anomalous cells in environment",
)

args = parser.parse_args()

env = DroneEnv(
    row_count=args.grid_size,
    col_count=args.grid_size,
    step_size=1.0,
    n_anamolous=args.n_anamolous,
    n_drones=args.n_drones,
)

state_size = env.state_size
action_size = env.action_size
n_drones = env.n_drones

actor_critic = Policy(
    (state_size + n_drones * 2,), (action_size,), base_kwargs={"recurrent": True}
)
actor_critic.load_state_dict(torch.load(args.model_path))

recurrent_hidden_states = torch.zeros(1, actor_critic.recurrent_hidden_state_size)
masks = torch.zeros(1, 1)

obs = env.reset()
done = False

step = 0
print(f"Step: {step+1}")
print(f"Drone positions:{env.n_drones_pos}")
temp_st = obs.reshape((5, 5))
print(temp_st, "\n")

drone_pos = env.n_drones_pos
obs = np.concatenate((obs, drone_pos[0]), -1)
obs = torch.tensor(obs, dtype=torch.float)

while not done:
    with torch.no_grad():
        value, action, _, recurrent_hidden_states = actor_critic.act(
            obs.unsqueeze(0), recurrent_hidden_states, masks, deterministic=True
        )

    obs, reward, done = env.step([action.item()])

    if (step + 1) % 1000 == 0:
        print(f"Step: {step+1}")
        print(f"Action: {action.item()}")
        print(f"Drone positions:{env.n_drones_pos}")
        temp_st = obs.reshape((5, 5))
        print(temp_st, "\n")

    drone_pos = env.n_drones_pos
    obs = np.concatenate((obs, drone_pos[0]), -1)
    obs = torch.tensor(obs, dtype=torch.float)

    masks.fill_(0.0 if done else 1.0)

    step += 1
