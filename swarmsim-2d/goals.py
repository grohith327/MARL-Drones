import arcade


class NormalGoal(arcade.Sprite):
    def __init__(self, filename, scale, stay_normal_iterations):

        super().__init__(filename, scale)
        self.wait_steps = stay_normal_iterations
        self.stay_normal_iterations = stay_normal_iterations

    def reduce_wait_steps(self):
        self.wait_steps -= 1

    def reset_wait_steps(self):
        self.wait_steps = self.stay_normal_iterations

    def get_wait_steps(self):
        return self.wait_steps

    def update(self):
        pass


class AnomalousGoal(arcade.Sprite):
    def __init__(self, filename, scale, stay_anomalous_iterations):

        super().__init__(filename, scale)
        self.wait_steps = stay_anomalous_iterations
        self.stay_anomalous_iterations = stay_anomalous_iterations

    def reduce_wait_steps(self):
        self.wait_steps -= 1

    def reset_wait_steps(self):
        self.wait_steps = self.stay_anomalous_iterations

    def get_wait_steps(self):
        return self.wait_steps

    def update(self):
        pass
