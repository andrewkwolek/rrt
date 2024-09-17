import matplotlib.pyplot as plt
import matplotlib.patches as pch
import numpy as np
import random

from obstacles import Obstacles
from tree import Tree, Node
from rrt import RRT
from matplotlib.collections import LineCollection

def main():
    # Generate random obstacles, q_init, and q_goal
    np.random.seed(42)
    fig, ax = plt.subplots()
    obstacles = Obstacles(100)
    obstacles.create_set(ax)
    q_init = Node((random.randrange(100), random.randrange(100)))
    q_goal = Node((random.randrange(100), random.randrange(100)))

    # Check if q_init or q_goal are in collision with an obstacle.
    # Generate a new q_init or q_goal if they are in collision
    while obstacles.check_collision(q_init.get_vertex()):
        q_init = Node((random.randrange(100), random.randrange(100)))
    while obstacles.check_collision(q_goal.get_vertex()):
        q_goal = Node((random.randrange(100), random.randrange(100)))

    # Create the RRT and run the finder
    rrt = RRT(q_init, q_goal, 1000, 1, 100)
    rrt.run_rrt(obstacles)

    # Trace the path from the goal back to the start
    path = rrt.trace_path()

    # Plot the RRT on matplotlib
    rrt.plot(ax)
    print(path)
    ax.axis((0, 100, 0, 100))
    ax.grid()
    plt.show()
    

if __name__=="__main__":
    main()