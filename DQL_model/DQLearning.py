import tensorflow as tf
import numpy as np
import random
from collections import deque
from noVis_sim import DroneEnv
from tensorflow.keras import layers

EPISODES = 300
BATCH_SIZE = 32


class AgentModel(tf.keras.Model):
    def __init__(self, state_size, n_drones, action_size):
        super(AgentModel, self).__init__()
        self.state_input = layers.Dense(128, activation="relu", input_dim=state_size)
        self.drone_input = layers.Dense(128, activation="relu", input_dim=n_drones * 2)
        self.dense1 = layers.Dense(256, activation="relu")
        self.dense2 = layers.Dense(128, activation="relu")
        self.dense3 = layers.Dense(64, activation="relu")
        self.dense4 = layers.Dense(32, activation="relu")
        self.dense5 = layers.Dense(action_size)

    def call(self, state, drone_pos):
        state_embed = self.state_input(state)
        drone_embed = self.drone_input(drone_pos)
        out = tf.concat([state_embed, drone_embed], axis=-1)
        out = self.dense1(out)
        out = self.dense2(out)
        out = self.dense3(out)
        out = self.dense4(out)
        out = self.dense5(out)
        return out


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
        self.learning_rate = 0.0001

        self.loss_object = None
        self.optimizer = None
        self.model = self._build_model()

    def _build_model(self):

        model = AgentModel(self.state_size, self.n_drones, self.action_size)
        self.loss_object = tf.keras.losses.MeanSquaredError()
        self.optimizer = tf.keras.optimizers.Adam(learning_rate=self.learning_rate)
        return model

    def _fit(self, state, drone_pos, target):
        with tf.GradientTape() as tape:
            preds = self.model(state, drone_pos)
            loss = self.loss_object(target, preds)
        gradients = tape.gradient(loss, self.model.trainable_variables)
        self.optimizer.apply_gradients(zip(gradients, self.model.trainable_variables))

    def memorize(
        self, state, drone_pos, action, reward, next_state, next_drone_pos, done
    ):
        self.memory.append(
            (state, drone_pos, action, reward, next_state, next_drone_pos, done)
        )

    def act(self, state, drone_pos, infer=False):
        actions = []
        rand_val = np.random.rand()
        for _ in range(self.n_drones):
            if infer:
                act_values = self.model(state, drone_pos)
                actions.append(np.argmax(act_values[0]))
                continue
            if rand_val <= self.epsilon:
                actions.append(random.randrange(0, self.action_size - 1))
            else:
                act_values = self.model(state, drone_pos)
                actions.append(np.argmax(act_values[0]))
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
            target = reward
            if not done:
                target = reward + self.gamma * np.amax(
                    self.model(next_state, next_drone_pos)[0]
                )
            target_f = self.model(state, drone_pos)
            target_f = target_f.numpy()
            target_f[0][action] = target
            self._fit(state, drone_pos, target_f)
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
done = False

for e in range(EPISODES):
    state = env.reset()
    state = np.reshape(state, [1, state_size])
    drone_pos = np.array(env.n_drones_pos)
    drone_pos = drone_pos.reshape((1, n_drones * 2))
    for time in range(500):
        action = agent.act(state, drone_pos)
        next_state, reward, done = env.step(action)
        reward = reward if not done else 1000
        next_state = np.reshape(next_state, [1, state_size])
        next_drone_pos = np.array(env.n_drones_pos)
        next_drone_pos = next_drone_pos.reshape((1, n_drones * 2))
        agent.memorize(
            state, drone_pos, action, reward, next_state, next_drone_pos, done
        )
        state = next_state
        drone_pos = next_drone_pos
        print(
            "episode: {}/{}, reward: {}, e: {:.2}".format(
                e, EPISODES, reward, agent.epsilon
            )
        )
        if done:
            break
        if len(agent.memory) > BATCH_SIZE:
            agent.replay(BATCH_SIZE)
        if e % 10:
            agent.save("DQModel_new.h5")
