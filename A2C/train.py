import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
from torch.distributions import Categorical
from tqdm.auto import tqdm
import logging
from curiosity import ICM
from rollouts import RolloutStorage


def prepare_inputs(args, obs, drone_pos, n_drones, obs_size):
    if args.policy == "MLP":
        obs = np.reshape(obs, [1, obs_size])

    if args.policy == "CNN" or args.policy == "Attn":
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

    icm_model_name = "ICM_" if args.enable_icm else ""

    log_file = f"A2C_{icm_model_name}{args.policy}.log"
    logging.basicConfig(filename=log_file, level=logging.INFO, format="%(message)s")

    steps = []
    total_steps = 0
    ep_rewards = 0.0
    grad_step = 0

    if args.enable_icm:
        icm = ICM(obs_size=obs_size, action_space=env.action_size)

    pbar = tqdm(total=args.total_steps)
    while total_steps < args.total_steps:

        obs = env.reset()
        drone_pos = np.array(env.n_drones_pos)

        obs, drone_pos = prepare_inputs(args, obs, drone_pos, n_drones, obs_size)
        curr_state = obs
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
            next_state = obs
            avg_rewards.append(rewards)
            if args.enable_icm:
                ## ICM for one drone
                rewards += icm(
                    a_t=torch.tensor(actions[0], dtype=torch.long),
                    a_t_logits=policies[0].detach(),
                    s_t=curr_state,
                    s_t1=next_state,
                )

            # reset the LSTM state for done envs
            masks = (
                1.0 - torch.from_numpy(np.array([dones], dtype=np.float32))
            ).unsqueeze(1)

            total_steps += 1
            pbar.update(1)

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
            + entropy_loss * args.entropy_coeff
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
                    f"A2C_models/{args.policy}_policy/A2C_drone_{icm_model_name}{i}.bin",
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


def agent_update(args, rollouts, actor_critic, optimizer):

    values, action_log_probs, dist_entropy, _ = actor_critic.evaluate_actions(
        rollouts.obs[:-1],
        rollouts.recurrent_hidden_states[0].unsqueeze(0),
        rollouts.masks[:-1].view(-1, 1),
        rollouts.actions.unsqueeze(0),
    )

    advantages = rollouts.returns[:-1] - values
    value_loss = advantages.pow(2).mean()
    action_loss = -(advantages.detach() * action_log_probs).mean()

    loss = (
        value_loss * args.value_loss_coef
        + action_loss
        - dist_entropy * args.entropy_coef
    )

    loss.backward()
    optimizer.step()

    return loss.item()


def learn(args, actor_critics, optimizers, env, obs_size, n_drones):

    log_file = f"A2C_new_policy.log"
    logging.basicConfig(filename=log_file, level=logging.INFO, format="%(message)s")

    rollouts = [
        RolloutStorage(
            num_steps=args.rollout_steps,
            obs_size=obs_size + 2,
            recurrent_hidden_state_size=actor_critics[i].recurrent_hidden_state_size,
        )
        for i in range(n_drones)
    ]

    pbar = tqdm(range(args.total_steps))
    for j in pbar:

        obs = env.reset()
        drone_pos = env.n_drones_pos
        # obs = np.concatenate((obs, drone_pos[0]), -1)
        # obs = torch.tensor(obs, dtype=torch.float)
        for i in range(n_drones):
            rollouts[i].obs[0].copy_(
                torch.tensor(np.concatenate((obs, drone_pos[i]), -1), dtype=torch.float)
            )

        eps_rewards = []

        for step in range(args.rollout_steps):

            values = []
            actions = []
            action_log_probs = []
            recurrent_hidden_states = []
            env_actions = []
            for i in range(n_drones):
                with torch.no_grad():
                    (
                        value,
                        action,
                        action_log_prob,
                        recurrent_hidden_state,
                    ) = actor_critics[i].act(
                        rollouts[i].obs[step].unsqueeze(0),
                        rollouts[i].recurrent_hidden_states[step].unsqueeze(0),
                        rollouts[i].masks[step].unsqueeze(0),
                    )
                values.append(value)
                actions.append(action)
                env_actions.append(action.item())
                action_log_probs.append(action_log_prob)
                recurrent_hidden_states.append(recurrent_hidden_state)

            obs, reward, done = env.step(env_actions)
            eps_rewards.append(reward)
            drone_pos = env.n_drones_pos
            # obs = np.concatenate((obs, drone_pos[0]), -1)

            # obs = torch.tensor(obs, dtype=torch.float)
            reward = torch.tensor(reward, dtype=torch.float)
            masks = torch.tensor([0.0 if done else 1.0], dtype=torch.float)

            for i in range(n_drones):
                rollouts[i].insert(
                    torch.tensor(
                        np.concatenate((obs, drone_pos[i]), -1), dtype=torch.float
                    ),
                    recurrent_hidden_states[i].squeeze(),
                    actions[i].squeeze(),
                    action_log_probs[i].squeeze(),
                    values[i].squeeze(),
                    reward,
                    masks,
                )

        losses = []
        for i in range(n_drones):
            with torch.no_grad():
                next_value = actor_critics[i].get_value(
                    rollouts[i].obs[-1].unsqueeze(0),
                    rollouts[i].recurrent_hidden_states[-1].unsqueeze(0),
                    rollouts[i].masks[-1].unsqueeze(0),
                )

            rollouts[i].compute_returns(next_value, args.gamma, args.gae_lambda)

            loss_value = agent_update(
                args, rollouts[i], actor_critics[i], optimizers[i]
            )
            losses.append(loss_value)
            rollouts[i].after_update()

        pbar.set_postfix(
            loss=np.mean(loss_value),
            avg_reward=np.mean(eps_rewards),
            min_reward=np.min(eps_rewards),
            max_reward=np.max(eps_rewards),
        )

        if (j + 1) % args.save_freq == 0:
            for i in range(n_drones):
                torch.save(
                    actor_critics[i].state_dict(),
                    f"A2C_models/{args.policy}_policy/A2C_drone_MARL_{i}.bin",
                )

