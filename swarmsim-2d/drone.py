import arcade


class Drone(arcade.Sprite):
    def __init__(self, filename, scale, grid, num, margin, width, height, row_count, col_count):

        super().__init__(filename, scale)
        self.grid = grid
        self.drone_num = num

        self.goal_coords = []
        for row in range(row_count):
            temp = []
            for col in range(col_count):
                x = (margin + width) * col + margin + width // 2
                y = (margin + height) * row + margin + height // 2
                temp.append([x, y])
            self.goal_coords.append(temp)
        
        self.row_count = row_count
        self.col_count = col_count

    def update_grid(self, row, col):
        self.grid[row][col] = -1

    def eucledian_dist(self, a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** (0.5)

    def update(self):
        closest_pos = None
        min_dist = None
        for row in range(self.row_count):
            for col in range(self.col_count):
                if self.grid[row][col] == -1:
                    continue
                dist = self.eucledian_dist(
                    [self.center_x, self.center_y], self.goal_coords[row][col]
                )
                if not min_dist:
                    min_dist = dist
                    closest_pos = self.goal_coords[row][col]
                    continue
                if min_dist > dist:
                    min_dist = dist
                    closest_pos = self.goal_coords[row][col]

        if (not closest_pos) and (not min_dist):
            return 1

        else:
            if self.center_x > closest_pos[0]:
                self.center_x -= 1
            if self.center_x < closest_pos[0]:
                self.center_x += 1
            if self.center_y > closest_pos[1]:
                self.center_y -= 1
            if self.center_y < closest_pos[1]:
                self.center_y += 1
