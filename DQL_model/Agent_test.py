import tensorflow as tf
import numpy as np
import random
from collections import deque
from noVis_sim import DroneEnv
from tensorflow.keras import layers
import os


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

    def act(self, state, infer=False):
        actions = []
        for _ in range(self.n_drones):
            if infer:
                act_values = self.model.predict(state)
                actions.append(np.argmax(act_values[0]))
                continue
            if np.random.rand() <= self.epsilon:
                actions.append(random.randrange(self.action_size))
            else:
                act_values = self.model.predict(state)
                actions.append(np.argmax(act_values[0]))
        return actions

    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
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

    def load(self, name):
        self.model.load_weights(name)


env = DroneEnv(row_count=10, col_count=10)

state_size = env.state_size
action_size = env.action_size
n_drones = env.n_drones

agent = Agent(state_size, action_size, n_drones)
agent.load("DQModel_tweaks.h5")
done = False

state = env.reset()
state = np.reshape(state, [1, state_size])

i = 0
print(f"Step: {i+1}")
print(f"Drone positions:{env.n_drones_pos}")
print(state, "\n")

if os.path.exists("drone_pos_tweaks.txt"):
    os.remove("drone_pos_tweaks.txt")
dronewriter = open("drone_pos_tweaks.txt", "a")
temp = env.n_drones_pos
temp = ",".join(list(map(lambda x: str(x), temp)))
dronewriter.write(temp + "\n")

if os.path.exists("state_tweaks.txt"):
    os.remove("state_tweaks.txt")
statewriter = open("state_tweaks.txt", "a")
temp = state
temp = temp.reshape(10, 10)
temp = ",".join(list(map(lambda x: str(x), temp)))
statewriter.write(temp + "\n")

if os.path.exists("uncertrain_cells_tweaks.txt"):
    os.remove("uncertrain_cells_tweaks.txt")
uncer_writer = open("uncertrain_cells_tweaks.txt", "a")
points = list(env.uncertain_points.keys())
points = ",".join(list(map(lambda x: str(x), points)))
uncer_writer.write(points + "\n")
uncer_writer.close()


while not done:
    action = agent.act(state, infer=True)
    next_state, reward, done = env.step(action)
    next_state = np.reshape(next_state, [1, state_size])
    state = next_state
    if done:
        break

    temp = env.n_drones_pos
    temp = ",".join(list(map(lambda x: str(x), temp)))
    dronewriter.write(temp + "\n")

    temp = state
    temp = temp.reshape(10, 10)
    temp = ",".join(list(map(lambda x: str(x), temp)))
    statewriter.write(temp + "\n")

    if (i + 1) % 10 == 0:
        print(f"Step: {i+1}")
        print(f"Drone positions:{env.n_drones_pos}")
        print(state, "\n")

    i += 1

print(f"Step: {i+1}")
print(f"Drone positions:{env.n_drones_pos}")
print(state, "\n")

dronewriter.close()
statewriter.close()