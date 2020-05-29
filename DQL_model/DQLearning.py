import tensorflow as tf
import numpy as np
import random
from collections import deque
from noVis_sim import DroneEnv
from tensorflow.keras import layers

EPISODES = 1000
BATCH_SIZE = 32


class Agent:
    def __init__(self, state_size, action_size, n_drones):
        self.state_size = state_size
        self.action_size = action_size
        self.n_drones = n_drones
        self.memory = deque(maxlen=2000)
        self.gamma = 0.95
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.model = self._build_model()

    def _build_model(self):
        model = tf.keras.Sequential()
        model.add(layers.Dense(128, activation="relu", input_dim=self.state_size))
        model.add(layers.Dense(64, activation="relu"))
        model.add(layers.Dense(32, activation="relu"))
        model.add(layers.Dense(self.action_size))

        model.compile(
            loss="mse",
            optimizer=tf.keras.optimizers.Adam(learning_rate=self.learning_rate),
        )
        return model

    def memorize(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        actions = []
        for _ in range(self.n_drones):
            if np.random.rand() <= self.epsilon:
                actions.append(random.randrange(self.action_size))
            else:
                act_values = self.model.predict(state)
                actions.append(np.argmax(act_values[0]))
        return actions

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            # print(rewards)
            # for reward in rewards:
            target = reward
            if not done:
                target = reward + self.gamma * np.amax(
                    self.model.predict(next_state)[0]
                )
            target_f = self.model.predict(state)
            target_f[0][action] = target
            self.model.fit(state, target_f, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def save(self, name):
        self.model.save_weights(name)


env = DroneEnv()

state_size = env.state_size
action_size = env.action_size
n_drones = env.n_drones

agent = Agent(state_size, action_size, n_drones)
done = False

for e in range(EPISODES):
    state = env.reset()
    state = np.reshape(state, [1, state_size])
    for time in range(500):
        action = agent.act(state)
        next_state, reward, done = env.step(action)
        reward = reward if not done else -1000
        next_state = np.reshape(next_state, [1, state_size])
        agent.memorize(state, action, reward, next_state, done)
        state = next_state
        print(
            "episode: {}/{}, score: {}, e: {:.2}".format(
                e, EPISODES, time, agent.epsilon
            )
        )
        if done:
            break
        if len(agent.memory) > BATCH_SIZE:
            agent.replay(BATCH_SIZE)
        if e % 10:
            agent.save("DQModel.h5")

