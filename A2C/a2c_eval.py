import torch
import numpy as np
import argparse
from noVis_sim import DroneEnv
import sys
import os

sys.path.insert(0, "pytorch-a2c-ppo-acktr-gail/")
from a2c_ppo_acktr.model import Policy


def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ("yes", "true", "t", "y", "1"):
        return True
    elif v.lower() in ("no", "false", "f", "n", "0"):
        return False
    else:
        raise argparse.ArgumentTypeError("Boolean value expected.")


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
parser.add_argument(
    "--log", type=str2bool, default=False, help="Log output in .txt file"
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

actor_critics = []
for i in range(n_drones):
    actor_critic = Policy(
        (state_size + 2,), (action_size,), base_kwargs={"recurrent": True}
    )
    actor_critic.load_state_dict(
        torch.load(args.model_path + f"A2C_drone_MARL_{i}.bin")
    )
    actor_critics.append(actor_critic)

recurrent_hidden_states = torch.zeros(1, actor_critic.recurrent_hidden_state_size)
masks = torch.zeros(1, 1)

obs = env.reset()
done = False

step = 0
print(f"Step: {step+1}")
print(f"Drone positions:{env.n_drones_pos}")
temp_st = obs.reshape((args.grid_size, args.grid_size))
print(temp_st, "\n")

if args.log:
    if os.path.exists("drone_pos_a2c.txt"):
        os.remove("drone_pos_a2c.txt")
    dronewriter = open("drone_pos_a2c.txt", "a")
    temp = env.n_drones_pos.copy()
    temp = ",".join(list(map(lambda x: str(x), temp)))
    dronewriter.write(temp + "\n")

    if os.path.exists("state_a2c.txt"):
        os.remove("state_a2c.txt")
    statewriter = open("state_a2c.txt", "a")
    temp = obs.copy()
    temp = temp.reshape(args.grid_size, args.grid_size)
    temp = ",".join(list(map(lambda x: str(x), temp)))
    statewriter.write(temp + "\n")

    if os.path.exists("uncertrain_cells_a2c.txt"):
        os.remove("uncertrain_cells_a2c.txt")
    uncer_writer = open("uncertrain_cells_a2c.txt", "a")
    points = list(env.uncertain_points.keys())
    points = ",".join(list(map(lambda x: str(x), points)))
    uncer_writer.write(points + "\n")
    uncer_writer.close()

drone_pos = env.n_drones_pos

while not done:
    actions = []
    for i in range(n_drones):
        with torch.no_grad():
            value, action, _, recurrent_hidden_states = actor_critics[i].act(
                torch.tensor(
                    np.concatenate((obs, drone_pos[i]), -1), dtype=torch.float
                ).unsqueeze(0),
                recurrent_hidden_states,
                masks,
            )
        actions.append(action.item())

    obs, reward, done = env.step(actions)

    if args.log:
        temp = env.n_drones_pos.copy()
        temp = ",".join(list(map(lambda x: str(x), temp)))
        dronewriter.write(temp + "\n")

        temp = obs.copy()
        temp = temp.reshape(args.grid_size, args.grid_size)
        temp = ",".join(list(map(lambda x: str(x), temp)))
        statewriter.write(temp + "\n")

    if (step + 1) % 1000 == 0:
        print(f"Step: {step+1}")
        print(f"Action: {actions}")
        print(f"Drone positions:{env.n_drones_pos}")
        temp_st = obs.reshape((args.grid_size, args.grid_size))
        print(temp_st, "\n")

    drone_pos = env.n_drones_pos
    masks.fill_(0.0 if done else 1.0)

    step += 1

if args.log:
    dronewriter.close()
    statewriter.close()
