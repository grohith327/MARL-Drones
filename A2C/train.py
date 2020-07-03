import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
from torch.distributions import Categorical
from tqdm.auto import tqdm
import logging


def prepare_inputs(args, obs, drone_pos, n_drones, obs_size):
    if args.policy == "MLP":
        obs = np.reshape(obs, [1, obs_size])

    if args.policy == "CNN":
        obs = np.reshape(obs, [1, args.grid_size, args.grid_size])

    if torch.is_tensor(obs):
        obs = Variable(obs).float()
    else:
        obs = Variable(torch.from_numpy(obs).float())

    if torch.is_tensor(drone_pos):
        drone_pos = drone_pos.view(n_drones, 2)
        drone_pos = Variable(drone_pos).float().view(1, -1)
    else:
        drone_pos = drone_pos.reshape((n_drones, 2))
        drone_pos = Variable(torch.from_numpy(drone_pos).float().view(1, -1))
    return obs, drone_pos


def train(args, nets, optimizers, env, obs_size, n_drones):

    log_file = f"A2C_{args.policy}.log"
    logging.basicConfig(filename=log_file, level=logging.INFO, format="%(message)s")

    steps = []
    total_steps = 0
    ep_rewards = 0.0
    grad_step = 0

    obs = env.reset()
    drone_pos = np.array(env.n_drones_pos)

    pbar = tqdm(total=args.total_steps)
    while total_steps < args.total_steps:

        obs, drone_pos = prepare_inputs(args, obs, drone_pos, n_drones, obs_size)
        avg_rewards = []

        for _ in range(args.rollout_steps):

            # network forward pass
            policies = []
            values = []
            actions = []
            for i in range(n_drones):
                p, v = nets[i](obs, drone_pos)
                probs = F.softmax(p, dim=-1)
                a = Categorical(probs).sample()[0]

                policies.append(p)
                values.append(v)
                actions.append(a.detach().unsqueeze(0).numpy())

            # gather env data, reset done envs and update their obs
            obs, rewards, dones = env.step(actions)
            ep_rewards += rewards
            if dones:
                ep_rewards = 0.0
                obs = env.reset()
            drone_pos = np.array(env.n_drones_pos)
            obs, drone_pos = prepare_inputs(args, obs, drone_pos, n_drones, obs_size)

            # reset the LSTM state for done envs
            masks = (
                1.0 - torch.from_numpy(np.array([dones], dtype=np.float32))
            ).unsqueeze(1)

            total_steps += 1
            pbar.update(1)

            avg_rewards.append(rewards)
            rewards = torch.tensor([rewards]).float().unsqueeze(1)

            actions = torch.tensor(actions)
            policies = torch.cat(policies)
            values = torch.cat(values)
            steps.append((rewards, masks, actions, policies, values))

        final_obs = obs
        final_drone_pos = drone_pos
        final_values = []
        for i in range(n_drones):
            _, final_v = nets[i](final_obs, final_drone_pos)
            final_values.append(final_v)

        final_values = torch.cat(final_values)
        steps.append((None, None, None, None, final_values))

        actions, policies, values, returns, advantages = process_rollout(args, steps)

        probs = F.softmax(policies, dim=-1)
        log_probs = F.log_softmax(policies, dim=-1)
        log_action_probs = log_probs.clone()

        policy_loss = (-log_action_probs * Variable(advantages)).mean()
        value_loss = advantages.pow(2).mean()
        entropy_loss = (-log_probs * probs).mean()

        loss = (
            policy_loss
            + value_loss * args.value_coeff
            - entropy_loss * args.entropy_coeff
        )

        loss.backward()

        if (grad_step + 1) % args.grad_acc == 0:
            for i in range(n_drones):
                torch.nn.utils.clip_grad_norm_(
                    nets[i].parameters(), args.grad_norm_limit
                )
                optimizers[i].step()
                optimizers[i].zero_grad()
        grad_step += 1

        steps = []

        if total_steps % args.save_freq == 0:
            for i in range(n_drones):
                torch.save(
                    nets[i].state_dict(),
                    f"A2C_models/{args.policy}_policy/A2C_drone_{i}.bin",
                )

        pbar.set_postfix(loss=loss.item(), reward=np.mean(avg_rewards))

        logging.info(f"loss: {loss.item()}, reward: {np.mean(avg_rewards)}")


def process_rollout(args, steps):
    # bootstrap discounted returns with final value estimates
    _, _, _, _, last_values = steps[-1]
    returns = last_values

    advantages = torch.zeros(1, 1)

    out = [None] * (len(steps) - 1)

    # run Generalized Advantage Estimation, calculate returns, advantages
    for t in reversed(range(len(steps) - 1)):
        rewards, masks, actions, policies, values = steps[t]
        # _, _, _, _, next_values = steps[t + 1]

        returns = rewards + returns * args.gamma * masks

        # deltas = rewards + next_values.data * args.gamma * masks - values.data
        # advantages = advantages * args.gamma * args.lambd * masks + deltas
        advantages = returns - values

        out[t] = actions, policies, values, returns, advantages

    # return data as batched Tensors, Variables
    return map(lambda x: torch.cat(x, 0), zip(*out))
