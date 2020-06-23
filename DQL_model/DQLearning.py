import numpy as np
import random
from collections import deque
from noVis_sim import DroneEnv
import torch
import torch.nn as nn
import torch.nn.functional as F

EPISODES = 1000
BATCH_SIZE = 32


class AgentModel(nn.Module):
    def __init__(self, state_size, n_drones):
        super(AgentModel, self).__init__()
        self.state_input = nn.Linear(state_size, 64)
        self.drone_input = nn.Linear(2 * n_drones, 64)
        self.dense1 = nn.Linear(128, 64)
        self.dense2 = nn.Linear(64, state_size)

    def forward(self, state, drone_pos):
        state = torch.tensor(state, dtype=torch.float)
        drone_pos = drone_pos.flatten()
        drone_pos = torch.tensor(drone_pos, dtype=torch.float).unsqueeze(0)
        state_embed = F.leaky_relu(self.state_input(state))
        drone_embed = F.leaky_relu(self.drone_input(drone_pos))
        out = torch.cat([state_embed, drone_embed], dim=-1)
        out = F.leaky_relu(self.dense1(out))
        out = self.dense2(out)
        return out


class Agent:
    def __init__(self, state_size, action_size, n_drones, row_count, col_count):
        self.state_size = state_size
        self.action_size = action_size
        self.n_drones = n_drones
        self.row_count = row_count
        self.col_count = col_count
        self.memory = deque(maxlen=2000)
        self.gamma = 0.95
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.0001

        self.mse_loss = None
        self.models, self.optimizers = self._build_model()

    def _build_model(self):

        self.mse_loss = nn.MSELoss()
        models = [
            AgentModel(self.state_size, self.n_drones) for _ in range(self.n_drones)
        ]
        optimizers = [
            torch.optim.AdamW(models[i].parameters(), lr=self.learning_rate)
            for i in range(self.n_drones)
        ]
        return models, optimizers

    def _fit(self, model, optimizer, state, drone_pos, target):
        model.zero_grad()
        target = torch.tensor(target, dtype=torch.float)
        preds = model(state, drone_pos)
        loss = self.mse_loss(preds, target)
        loss.backward()
        optimizer.step()

    def memorize(
        self, state, drone_pos, action, reward, next_state, next_drone_pos, done
    ):
        self.memory.append(
            (state, drone_pos, action, reward, next_state, next_drone_pos, done)
        )

    def pos_to_act(self, curr_x, curr_y, target):
        target = target.reshape((self.row_count, self.col_count))
        target_x, target_y = np.where(target == np.max(target))
        target_x = target_x[0]
        target_y = target_y[0]
        if target_x - curr_x == 0 and target_y - curr_y == 0:
            return 4
        if target_y - curr_y < 0:
            return 0
        if target_y - curr_y > 0:
            return 2
        if target_x - curr_x > 0:
            return 1
        return 3

    def act(self, state, drone_pos, infer=False):
        actions = []
        rand_val = np.random.rand()
        for i in range(self.n_drones):
            if infer:
                act_values = self.models[i](state, drone_pos)
                act_values = act_values.detach().numpy()
                actions.append(
                    self.pos_to_act(drone_pos[i][0], drone_pos[i][1], act_values[0])
                )
                continue
            if rand_val <= self.epsilon:
                actions.append(random.randrange(0, self.action_size - 1))
            else:
                act_values = self.models[i](state, drone_pos)
                act_values = act_values.detach().numpy()
                actions.append(
                    self.pos_to_act(drone_pos[i][0], drone_pos[i][1], act_values[0])
                )
        return actions

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for (
            state,
            drone_pos,
            action,
            reward,
            next_state,
            next_drone_pos,
            done,
        ) in minibatch:
            for i in range(self.n_drones):
                target = reward
                if not done:
                    target = reward + self.gamma * np.amax(
                        self.models[i](next_state, next_drone_pos)[0].detach().numpy()
                    )
                target_f = self.models[i](state, drone_pos)
                target_f = target_f.detach().numpy()
                target_f[0][action] = target
                self._fit(
                    self.models[i], self.optimizers[i], state, drone_pos, target_f
                )
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def save(self, name):
        for i in range(self.n_drones):
            torch.save(self.models[i].state_dict(), f"{name}_drone_{i}.bin")

    def load(self, name):
        for i in range(self.n_drones):
            self.models[i].load_state_dict(torch.load(f"{name}_drone_{i}.bin"))


env = DroneEnv(row_count=5, col_count=5)

state_size = env.state_size
action_size = env.action_size
n_drones = env.n_drones
row_count = env.row_count
col_count = env.col_count

agent = Agent(state_size, action_size, n_drones, row_count, col_count)
done = False

for e in range(EPISODES):
    state = env.reset()
    state = np.reshape(state, [1, state_size])
    drone_pos = np.array(env.n_drones_pos)
    drone_pos = drone_pos.reshape((n_drones, 2))
    for time in range(1000):
        action = agent.act(state, drone_pos)
        next_state, reward, done = env.step(action)
        reward = reward if not done else 1000
        next_state = np.reshape(next_state, [1, state_size])
        next_drone_pos = np.array(env.n_drones_pos)
        next_drone_pos = next_drone_pos.reshape((n_drones, 2))
        agent.memorize(
            state, drone_pos, action, reward, next_state, next_drone_pos, done
        )
        state = next_state
        drone_pos = next_drone_pos
        print(
            "episode: {}/{}, timestep: {}/{}, reward: {}, e: {:.2}".format(
                e, EPISODES, time + 1, 1000, reward, agent.epsilon
            )
        )
        if done:
            break
        if len(agent.memory) > BATCH_SIZE:
            agent.replay(BATCH_SIZE)
        if e % 10:
            agent.save("DQL_envchanges_55_grid")
