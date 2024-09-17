import matplotlib.pyplot as plt
import matplotlib.patches as pch
import numpy as np

from tree import Tree, Node
from rrt import RRT
from matplotlib.collections import LineCollection

def generate_obstacles(axes):
    obstacles = []
    c = pch.Circle((25,25), 3, color="black")
    obstacles.append(c)
    plt.gca().add_patch(c)
    c = pch.Circle((60,30), 7, color="black")
    obstacles.append(c)
    plt.gca().add_patch(c)
    c = pch.Circle((20,75), 5, color="black")
    obstacles.append(c)
    plt.gca().add_patch(c)
    return obstacles

def main():
    np.random.seed(42)
    figure, axes = plt.subplots()
    obstacles = generate_obstacles(axes)
    q_init = Node((50, 50))
    q_goal = Node((70, 20))
    rrt = RRT(q_init, q_goal, 500, 1, 100)
    rrt.run_rrt(obstacles)

    plt.axis((0, 100, 0, 100))
    plt.grid()
    plt.show()
    

if __name__=="__main__":
    main()