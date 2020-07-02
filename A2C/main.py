import torch
import torch.optim as optim
from models import MlpPolicy, CNNPolicy
from noVis_sim import DroneEnv
from train import train
import argparse

parser = argparse.ArgumentParser(description="A2C (Advantage Actor-Critic)")
parser.add_argument("--rollout-steps", type=int, default=20, help="steps per rollout")
parser.add_argument(
    "--total-steps",
    type=int,
    default=int(1e6),
    help="total number of steps to train for",
)
parser.add_argument("--lr", type=float, default=1e-4, help="learning rate")
parser.add_argument("--gamma", type=float, default=0.99, help="gamma parameter for GAE")
parser.add_argument(
    "--lambd", type=float, default=1.00, help="lambda parameter for GAE"
)
parser.add_argument(
    "--value_coeff", type=float, default=0.5, help="value loss coeffecient"
)
parser.add_argument(
    "--entropy_coeff", type=float, default=0.01, help="entropy loss coeffecient"
)
parser.add_argument(
    "--grad_norm_limit",
    type=float,
    default=10.0,
    help="gradient norm clipping threshold",
)
parser.add_argument("--seed", type=int, default=0, help="random seed")
parser.add_argument(
    "--save_freq", type=int, default=10, help="frequency to save model weights"
)
parser.add_argument("--grad_acc", type=int, default=1, help="grad accumulation steps")
parser.add_argument("--policy", type=str, default="MLP", help="CNN or MLP")
parser.add_argument("--grid_size", type=int, default=5, help="Grid size, Default: 5x5")
parser.add_argument("--n_drones", type=int, default=3, help="number of drones")
parser.add_argument(
    "--n_anamolous",
    type=int,
    default=5,
    help="number of anomalous cells in environment",
)

args = parser.parse_args()

assert args.policy == "CNN" or args.policy == "MLP", "policy must be CNN or MLP"

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

if args.policy == "MLP":
    nets = [MlpPolicy(state_size, n_drones, action_size) for _ in range(n_drones)]
else:
    nets = [CNNPolicy(n_drones, action_size) for _ in range(n_drones)]

optimizers = [optim.AdamW(nets[i].parameters(), lr=args.lr) for i in range(n_drones)]

train(
    args=args,
    nets=nets,
    optimizers=optimizers,
    env=env,
    obs_size=state_size,
    n_drones=n_drones,
)
