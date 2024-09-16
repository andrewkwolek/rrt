from tree import Tree, Node
import numpy as np
import random

class RRT():
    def __init__(self, q, K, d, D):
        self.q_init = q
        self.K = K
        self.delta = d
        self.D = D

        self.tree = Tree(self.q_init)

    def iterate_rrt(self):
        q_rand = Node((random.randrange(self.D), random.randrange(self.D)))
        min_dist = 99999
        q_near = None
        for v in self.tree.get_vertices():
            dist = np.linalg.norm(np.array(q_rand.vertex) - np.array(v.vertex))
            if dist < min_dist:
                q_near = v
                min_dist = dist
        
        vec = (np.array(q_rand.vertex) - np.array(q_near.vertex)) / min_dist * self.delta
        q_new = Node((q_near.vertex[0] + vec[0], q_near.vertex[1] + vec[1]))
        self.tree.insert_vertex(q_new)
        self.tree.insert_edges(q_new, q_near)

    def run_rrt(self):
        for i in range(self.K):
            self.iterate_rrt()
        return self.tree
