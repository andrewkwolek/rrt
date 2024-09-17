import matplotlib.pyplot as plt
import matplotlib.patches as pch
import numpy as np
import random

class Obstacles:
    np.random.seed(42)
    def __init__(self, D):
        self.obstacles = []
        self.D = D
        self.num_obs = 25

    def generate_obs(self, rad, axes):
        coords = (random.randrange(self.D), random.randrange(self.D))
        c = pch.Circle(coords, rad, color="black", zorder=2)
        self.obstacles.append(c)
        axes.add_artist(c)
    
    def create_set(self, axes):
        for _ in range(self.num_obs):
            rad = random.randrange(15)
            self.generate_obs(rad, axes)

    def check_collision(self, point):
        for o in self.obstacles:
            tf = o.get_data_transform().transform(point)
            if o.contains_point(tf):
                return True
        return False

    def get_obs(self):
        return self.obstacles