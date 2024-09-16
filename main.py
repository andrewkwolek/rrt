import matplotlib.pyplot as plt
import numpy as np

from tree import Tree, Node
from rrt import RRT
from matplotlib.collections import LineCollection

def main():
    np.random.seed(42)
    fig, ax = plt.subplots()
    plt.axis((0, 100, 0, 100))
    q_init = Node((50, 50))
    rrt = RRT(q_init, 500, 1, 100)
    rrt.run_rrt()

    x = []
    y = []
    for v in rrt.tree.get_vertices():
        x.append(v.vertex[0])
        y.append(v.vertex[1])

    plt.axis((0, 100, 0, 100))
    plt.scatter(x, y, s=2, color="blue")
    plt.grid()
    plt.show()
    

if __name__=="__main__":
    main()