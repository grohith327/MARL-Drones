import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse

parser = argparse.ArgumentParser(description="Drone Swarm Simulation")
parser.add_argument("--grid_size", type=int, default=5, help="Grid size, Default: 5x5")
parser.add_argument(
    "--refresh_rate", type=float, default=1, help="refresh rate of animation"
)
args = parser.parse_args()

row = args.grid_size
col = args.grid_size
drone_pos_file = "Visualization_files/drone_pos_a2c.txt"
state_file = "Visualization_files/state_a2c.txt"
anam_cells = [(2, 0), (3, 2), (2, 3), (3, 3), (1, 0)]  # Manually entered

env = []
for i in range(row):
    for j in range(col):
        env.append([i, j])

env = np.array(env)

anam_cells = np.array(anam_cells)

with open(drone_pos_file, "r") as f:
    data = f.read()
data = data.split("\n")
data = data[:-1]

drone1_pos = []
# drone2_pos = []
# drone3_pos = []

for line in data:
    line = line.split(",")
    x1 = float(line[0][1:])
    y1 = float(line[1][:-1])
    drone1_pos.append([x1, y1])


with open(state_file, "r") as f:
    data = f.read()
data = data.split("\n")
data = data[:-1]

states = []
for i in range(len(data)):
    temp = []
    t = data[i].split(",")
    for val in t:
        val = val.split()
        for i in val:
            i = i.replace("[", "")
            i = i.replace("]", "")
            temp.append(int(i))
    temp = np.array(temp)
    try:
        temp = temp.reshape(row, col)
        states.append(temp)
    except:
        print(i)
        break

states = np.array(states)

drone1_pos = np.array(drone1_pos)
# drone2_pos = np.array(drone2_pos)
# drone3_pos = np.array(drone3_pos)

fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(1, 1, 1)

i = 0


def animate(i):
    ax.clear()
    ax.set_xticks(np.arange(0, row + 1, 1.0))
    ax.set_yticks(np.arange(0, col + 1, 1.0))
    ax.set_xlim(-1, row)
    ax.set_ylim(0, col + 1)

    ax.scatter(env[:, 1], col - env[:, 0], marker="*", color="black", s=100)
    ax.scatter(anam_cells[:, 1], col - anam_cells[:, 0], marker="*", color="red", s=100)
    mapped = []
    state = states[i]
    for j in range(row):
        for k in range(col):
            if state[j][k] == 0:
                mapped.append([j, k])
    mapped = np.array(mapped)
    if mapped.shape[0] > 0:
        ax.scatter(mapped[:, 0], col - mapped[:, 1], marker="*", color="white", s=100)
    ax.scatter(
        drone1_pos[i, 0], col - drone1_pos[i, 1], marker="^", color="green", s=150
    )
    # ax.scatter(
    #     drone2_pos[i, 0], col - drone2_pos[i, 1], marker="^", color="green", s=150
    # )
    # ax.scatter(
    #     drone3_pos[i, 0], col - drone3_pos[i, 1], marker="^", color="green", s=150
    # )
    i += 1
    if i == len(drone1_pos):
        print("starting over")
        i = 0


ani = animation.FuncAnimation(fig, animate, interval=args.refresh_rate)
plt.show()
