import numpy as np
import random
import math
from functools import reduce


class DroneEnv:
    def __init__(
        self,
        row_count=20,
        col_count=20,
        n_anamolous=5,
        uncertainity=5,
        collision_dist=0.5,
        n_drones=3,
        step_size=0.2,
    ):
        self.grid = []
        self.row_count = row_count
        self.col_count = col_count
        self.n_anamolous = n_anamolous
        self.uncertainity = uncertainity
        self.collision_dist = collision_dist
        self.n_drones = n_drones
        self.n_drones_pos = []
        self.step_size = step_size

        self.action_size = 8
        self.state_size = row_count * col_count

        self.init_env()

    def init_env(self):

        self.grid = []
        for _ in range(self.row_count):
            self.grid.append([1] * self.col_count)

        i = 0
        while i < self.n_anamolous:
            a = np.random.randint(self.row_count)
            b = np.random.randint(self.col_count)
            if self.grid[a][b] == 1:
                self.grid[a][b] = 2 * self.uncertainity
                i += 1

        for _ in range(self.n_drones):
            x = random.uniform(0.0, float(self.row_count))
            y = random.uniform(0.0, float(self.col_count))

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

        for x in range(self.row_count):
            for y in range(self.col_count):
                dist = self._eucid_dist(drone_x, drones_y, x, y)
                if dist < self.collision_dist and self.grid[x][y] > 0:
                    self.grid[x][y] -= 1
                    return True

        return False

    def step(self, actions):

        assert len(actions) == self.n_drones, "provide actions for each drone"

        total_reward = 0
        scale_dist = 1 / math.sqrt(2)
        for i, action in enumerate(actions):
            assert action >= 0 and action < 8, "action should be in range:[0,8)"
            if action == 0:
                self.n_drones_pos[i][1] -= self.step_size
            elif action == 1:
                self.n_drones_pos[i][0] += self.step_size * scale_dist
                self.n_drones_pos[i][1] -= self.step_size * scale_dist
            elif action == 2:
                self.n_drones_pos[i][0] += self.step_size
            elif action == 3:
                self.n_drones_pos[i][0] += self.step_size * scale_dist
                self.n_drones_pos[i][1] += self.step_size * scale_dist
            elif action == 4:
                self.n_drones_pos[i][1] += self.step_size
            elif action == 5:
                self.n_drones_pos[i][0] -= self.step_size * scale_dist
                self.n_drones_pos[i][1] += self.step_size * scale_dist
            elif action == 6:
                self.n_drones_pos[i][0] -= self.step_size
            else:
                self.n_drones_pos[i][0] -= self.step_size * scale_dist
                self.n_drones_pos[i][1] -= self.step_size * scale_dist

            det_flag = self._det_collision(
                self.n_drones_pos[i][0], self.n_drones_pos[i][1]
            )
            if det_flag:
                total_reward += 5

        total_reward -= 1
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
