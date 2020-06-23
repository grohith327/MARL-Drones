import numpy as np
import random
import math
from functools import reduce


class DroneEnv:
    def __init__(
        self,
        row_count=10,
        col_count=10,
        n_anamolous=5,
        uncertainity=5,
        n_drones=3,
        step_size=1,
    ):
        self.grid = None
        self.row_count = row_count
        self.col_count = col_count
        self.n_anamolous = n_anamolous
        self.uncertainity = uncertainity
        self.uncertain_points = None
        self.n_drones = n_drones
        self.n_drones_pos = None
        self.step_size = step_size

        self.action_size = 5
        self.state_size = row_count * col_count

        self.step_func_count = 0

        self.init_env()

    def init_env(self):

        self.step_func_count = 0

        self.grid = []
        for _ in range(self.row_count):
            self.grid.append([1] * self.col_count)

        i = 0
        self.uncertain_points = {}
        while i < self.n_anamolous:
            a = np.random.randint(self.row_count)
            b = np.random.randint(self.col_count)
            if self.grid[a][b] == 1:
                self.grid[a][b] = self.uncertainity
                self.uncertain_points[(a, b)] = 1
                i += 1

        self.n_drones_pos = []
        self.prev_drone_pos = []
        for _ in range(self.n_drones):
            x = np.random.randint(self.row_count)
            y = np.random.randint(self.col_count)

            self.n_drones_pos.append([x, y])

    def reset(self):
        self.init_env()
        grid = self.grid.copy()
        grid = reduce(lambda x, y: x + y, grid)
        grid = np.array(grid)
        return grid

    def _eucid_dist(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def _det_collision(self, drone_x, drones_y):

        reward = 0.0

        if self.grid[drone_x][drones_y] == 0:
            reward -= 0.1
            return reward

        self.grid[drone_x][drones_y] -= 1
        reward += self._check_uncertain_mapped()
        return reward

    def _drone_dist(self):
        dist_count = 0
        for i in range(self.n_drones - 1):
            for j in range(i + 1, self.n_drones):
                x1 = self.n_drones_pos[i][0]
                y1 = self.n_drones_pos[i][1]
                x2 = self.n_drones_pos[j][0]
                y2 = self.n_drones_pos[j][1]
                if self._eucid_dist(x1, y1, x2, y2) > self.row_count // self.n_drones:
                    dist_count += 1
        return dist_count

    def _check_uncertain_mapped(self):
        reward = 0.0
        for k, v in self.uncertain_points.items():
            if self.grid[k[0]][k[1]] == 0 and self.uncertain_points[k] == 1:
                reward += 2.0
                self.uncertain_points[k] = 0
        return reward

    def step(self, actions):

        assert len(actions) == self.n_drones, "provide actions for each drone"

        total_reward = 0.0
        for i, action in enumerate(actions):
            assert (
                action >= 0 and action < self.action_size
            ), f"action should be in range:[0,{self.action_size})"
            if action == 0:
                self.n_drones_pos[i][1] -= self.step_size
            elif action == 1:
                self.n_drones_pos[i][0] += self.step_size
            elif action == 2:
                self.n_drones_pos[i][1] += self.step_size
            elif action == 3:
                self.n_drones_pos[i][0] -= self.step_size

            self.n_drones_pos[i][0] = np.clip(
                self.n_drones_pos[i][0], 0, self.row_count - 1
            )
            self.n_drones_pos[i][1] = np.clip(
                self.n_drones_pos[i][1], 0, self.col_count - 1
            )

            total_reward += self._det_collision(
                self.n_drones_pos[i][0], self.n_drones_pos[i][1]
            )

        total_reward -= (0.5) * (
            self.step_func_count
        )  ## Linearly increasing punishment as env runs
        total_reward += (0.2 * self._drone_dist()) * (1.01) ** (
            -self.step_func_count
        )  ## Exponentialy decreasing reward as drones spread out
        self.step_func_count += 1
        done = True
        for i in range(self.row_count):
            f = 0
            for j in range(self.col_count):
                if self.grid[i][j] > 0:
                    done = False
                    f = 1
                    break
            if f == 1:
                break

        grid = self.grid.copy()
        grid = reduce(lambda x, y: x + y, self.grid)
        grid = np.array(grid)

        return grid, total_reward, done
