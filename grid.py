import arcade
import random
import numpy as np
import argparse
from functools import reduce
import sys
import time

parser = argparse.ArgumentParser(description='Drone Swarm Simulation')
parser.add_argument('--num_drones', type = int, default=3, help='Total number of drones')
parser.add_argument('--num_anomalous_cells', type = int, default = 50, help = 'number of anomalous cells in the simulation')
parser.add_argument('--row_count', type = int, default=20, help='row count')
parser.add_argument('--col_count', type = int, default=20, help='column count')

args = parser.parse_args()

## Grid Params
WIDTH = 40
HEIGHT = 40
ROW_COUNT = args.row_count
COL_COUNT = args.col_count
MARGIN = 5

## Window params
SCREEN_WIDTH = (WIDTH + MARGIN) * COL_COUNT + MARGIN
SCREEN_HEIGHT = (HEIGHT + MARGIN) * ROW_COUNT + MARGIN
SCREEN_TITLE = "Drone SwarmSim"

## Drones params
SPRITE_SCALING_DRONE = 0.30
NUM_DRONES = args.num_drones

## Goal Params
SPRITE_NORMAL_SCALING_GOAL = 0.01
SPRITE_ANOMALOUS_SCALING_GOAL = 0.002
NUM_ANOMALOUS_CELLS = args.num_anomalous_cells

## Collision Params
STAY_NORMAL_ITERATIONS = 10
STAY_ANOMALOUS_ITERATION = 35
COLLISION_RADIUS = 30.0

class NormalGoal(arcade.Sprite):

    def __init__(self, filename, scale):

        super().__init__(filename, scale)
        self.wait_steps = STAY_NORMAL_ITERATIONS
    
    def reduce_wait_steps(self):
        self.wait_steps -= 1
    
    def reset_wait_steps(self):
        self.wait_steps = STAY_NORMAL_ITERATIONS
    
    def get_wait_steps(self):
        return self.wait_steps

    def update(self):
        pass

class AnomalousGoal(arcade.Sprite):

    def __init__(self, filename, scale):

        super().__init__(filename, scale)
        self.wait_steps = STAY_ANOMALOUS_ITERATION

    def reduce_wait_steps(self):
        self.wait_steps -= 1
    
    def reset_wait_steps(self):
        self.wait_steps = STAY_ANOMALOUS_ITERATION
    
    def get_wait_steps(self):
        return self.wait_steps

    def update(self):
        pass


class Drone(arcade.Sprite):

    def __init__(self, filename, scale, grid, num):

        super().__init__(filename, scale)
        self.grid = grid
        self.drone_num = num

        self.goal_coords = []
        for row in range(ROW_COUNT):
            temp = []
            for col in range(COL_COUNT):
                x = (MARGIN + WIDTH) * col + MARGIN + WIDTH // 2
                y = (MARGIN + HEIGHT) * row + MARGIN + HEIGHT // 2
                temp.append([x,y])
            self.goal_coords.append(temp)
    
    def update_grid(self, row, col):
        self.grid[row][col] = -1

    def eucledian_dist(self, a, b):
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**(0.5)

    def update(self):
        closest_pos = None
        min_dist = None
        for row in range(ROW_COUNT):
            for col in range(COL_COUNT):
                if(self.grid[row][col] == -1):
                    continue
                dist = self.eucledian_dist([self.center_x, self.center_y], self.goal_coords[row][col])
                if(not min_dist):
                    min_dist = dist
                    closest_pos = self.goal_coords[row][col]
                    continue
                if(min_dist > dist):
                    min_dist = dist
                    closest_pos = self.goal_coords[row][col]
        
        if((not closest_pos) and (not min_dist)):
            return 1
        
        else:
            if(self.center_x > closest_pos[0]):
                self.center_x -= 1
            if(self.center_x < closest_pos[0]):
                self.center_x += 1
            if(self.center_y > closest_pos[1]):
                self.center_y -= 1
            if(self.center_y < closest_pos[1]):
                self.center_y += 1


class GridWorld(arcade.Window):

    def __init__(self, screen_width, screen_height, title):
        super().__init__(screen_width, screen_height, title)

        arcade.set_background_color(arcade.color.BLACK)

        self.drone_sprite_list = None
        self.normal_goals_sprite_list = None
        self.anomalous_goals_sprite_list = None

        self.grid = []

        rand_nums = np.random.rand((ROW_COUNT*COL_COUNT))
        rand_nums = np.argsort(rand_nums)[:NUM_ANOMALOUS_CELLS]

        count = 0
        for _ in range(ROW_COUNT):
            temp = []
            for _ in range(COL_COUNT):
                if(count in rand_nums):
                    temp.append(1)
                else:
                    temp.append(0)
                count += 1
            self.grid.append(temp)
        

    def setup(self):

        self.drone_sprite_list = arcade.SpriteList()
        self.normal_goals_sprite_list = arcade.SpriteList()
        self.anomalous_goals_sprite_list = arcade.SpriteList()

        for i in range(NUM_DRONES):
            drone = Drone(":resources:images/space_shooter/playerShip1_orange.png", SPRITE_SCALING_DRONE, self.grid, i+1)
            drone.center_x = random.randrange(SCREEN_WIDTH - MARGIN)
            drone.center_y = random.randrange(SCREEN_HEIGHT - MARGIN)
            drone._set_collision_radius(COLLISION_RADIUS)
            self.drone_sprite_list.append(drone)
        
        for row in range(ROW_COUNT):
            for col in range(COL_COUNT):
                if(self.grid[row][col] == 1):
                    goal = AnomalousGoal('./resources/red.png', SPRITE_ANOMALOUS_SCALING_GOAL)
                    goal.center_x = (MARGIN + WIDTH) * col + MARGIN + WIDTH // 2
                    goal.center_y = (MARGIN + HEIGHT) * row + MARGIN + HEIGHT // 2
                    self.anomalous_goals_sprite_list.append(goal)
                else:
                    goal = NormalGoal('./resources/green.png', SPRITE_NORMAL_SCALING_GOAL)
                    goal.center_x = (MARGIN + WIDTH) * col + MARGIN + WIDTH // 2
                    goal.center_y = (MARGIN + HEIGHT) * row + MARGIN + HEIGHT // 2
                    self.normal_goals_sprite_list.append(goal)

    def on_draw(self):

        arcade.start_render()

        for row in range(ROW_COUNT):
            for col in range(COL_COUNT):

                x = (MARGIN + WIDTH) * col + MARGIN + WIDTH // 2
                y = (MARGIN + HEIGHT) * row + MARGIN + HEIGHT // 2

                arcade.draw_rectangle_filled(x, y, WIDTH, HEIGHT, color = arcade.color.WHITE)
        
        self.drone_sprite_list.draw()
        self.normal_goals_sprite_list.draw()
        self.anomalous_goals_sprite_list.draw()
    
    def on_update(self, delta_time):

        if(len(self.drone_sprite_list) == 0):
            time.sleep(1)
            sys.exit()

        ## Pull drones down when more than half of the states are mapped
        total = sum(reduce(lambda x,y :x+y, self.grid))
        kill_drone = None
        for i in range(len(self.drone_sprite_list) - 1):
            for j in range(i+1, len(self.drone_sprite_list)):
                if(abs(self.drone_sprite_list[i].center_x - self.drone_sprite_list[j].center_x) < COLLISION_RADIUS / 2 and abs(
                    self.drone_sprite_list[i].center_y - self.drone_sprite_list[j].center_y) < COLLISION_RADIUS / 2 and total < 0):
                    kill_drone = self.drone_sprite_list[i]
                    break
        
        if(kill_drone):
            kill_drone.remove_from_sprite_lists()

        for drone in self.drone_sprite_list:

            val = drone.update()
            if(val):
                drone.kill()

            anomalous_hit_list = arcade.check_for_collision_with_list(drone, self.anomalous_goals_sprite_list)

            normal_hit_list = arcade.check_for_collision_with_list(drone, self.normal_goals_sprite_list)
            
            for goal in normal_hit_list:
                if(goal.get_wait_steps() == 0):
                    goal.remove_from_sprite_lists()
                    col = (goal.center_x - MARGIN - WIDTH // 2) // ((MARGIN + WIDTH))
                    row = (goal.center_y - MARGIN - HEIGHT // 2) // ((MARGIN + HEIGHT))
                    self.grid[row][col] = -1
                    drone.update_grid(row, col)
                else:
                    goal.reduce_wait_steps()
            
            for goal in anomalous_hit_list:
                if(goal.get_wait_steps() == 0):
                    goal.remove_from_sprite_lists()
                    col = (goal.center_x - MARGIN - WIDTH // 2) // ((MARGIN + WIDTH))
                    row = (goal.center_y - MARGIN - HEIGHT // 2) // ((MARGIN + HEIGHT))
                    self.grid[row][col] = -1
                    drone.update_grid(row, col)
                else:
                    goal.reduce_wait_steps()

    
def main():
    assert NUM_ANOMALOUS_CELLS <= ROW_COUNT*COL_COUNT, "Number of anomalous cells has to less than total number of cells"
    assert NUM_DRONES < ROW_COUNT*COL_COUNT, "Number of drones has to less than total number of cells"
    game = GridWorld(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
    game.setup()
    arcade.run()

if __name__ == "__main__":
    main()