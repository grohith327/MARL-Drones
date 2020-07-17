import torch
import torch.optim as optim

# from models import Policy
from noVis_sim import DroneEnv
from train import train, learn
import argparse
import sys

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


parser = argparse.ArgumentParser(description="A2C (Advantage Actor-Critic)")
parser.add_argument(
    "--rollout_steps", type=int, default=25, help="number of rollout steps"
)
parser.add_argument(
    "--total-steps",
    type=int,
    default=int(1e6),
    help="total number of steps to train for",
)
parser.add_argument("--lr", type=float, default=2e-4, help="learning rate")
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
parser.add_argument("--policy", type=str, default="MLP", help="CNN, MLP, Attn policy")
parser.add_argument("--grid_size", type=int, default=5, help="Grid size, Default: 5x5")
parser.add_argument("--n_drones", type=int, default=3, help="number of drones")
parser.add_argument(
    "--n_anamolous",
    type=int,
    default=5,
    help="number of anomalous cells in environment",
)
parser.add_argument(
    "--enable_icm", type=str2bool, default=False, help="use intrinsic curiosity module"
)
parser.add_argument(
    "--gamma",
    type=float,
    default=0.99,
    help="discount factor for rewards (default: 0.99)",
)
parser.add_argument(
    "--gae-lambda",
    type=float,
    default=0.95,
    help="gae lambda parameter (default: 0.95)",
)
parser.add_argument(
    "--entropy-coef",
    type=float,
    default=0.01,
    help="entropy term coefficient (default: 0.01)",
)
parser.add_argument(
    "--value-loss-coef",
    type=float,
    default=0.5,
    help="value loss coefficient (default: 0.5)",
)

args = parser.parse_args()

assert args.policy in ["CNN", "MLP", "Attn"], "policy must be one of [CNN, MLP, Attn]"

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

# nets = [
#     Policy(state_size, n_drones, action_size, policy_type=args.policy)
#     for _ in range(n_drones)
# ]
# optimizers = [optim.AdamW(nets[i].parameters(), lr=args.lr) for i in range(n_drones)]

# train(
#     args=args,
#     nets=nets,
#     optimizers=optimizers,
#     env=env,
#     obs_size=state_size,
#     n_drones=n_drones,
# )


actor_critics = [
    Policy((state_size + 2,), (action_size,), base_kwargs={"recurrent": True})
    for _ in range(n_drones)
]
optimizers = [
    optim.RMSprop(actor_critics[i].parameters(), lr=args.lr) for i in range(n_drones)
]

learn(
    args=args,
    actor_critics=actor_critics,
    optimizers=optimizers,
    env=env,
    obs_size=state_size,
    n_drones=n_drones,
)
