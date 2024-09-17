from tree import Tree, Node
import numpy as np
import random

import matplotlib.pyplot as plt
import matplotlib.patches as pch

class RRT():
    def __init__(self, q_init, q_goal, K, d, D):
        self.q_init = q_init
        self.q_goal = q_goal
        self.K = K
        self.delta = d
        self.D = D

        self.tree = Tree(self.q_init)

    ################### START_CITATION [1] #################
    def minDist(self, a, b, p):
        ab = np.array([b[0] - a[0], b[1] - a[1]])
        ap = np.array([p[0] - a[0], p[1] - a[1]])
        bp = np.array([p[0] - b[0], p[1] - b[1]])

        ab_ap = np.dot(ab, ap)
        ab_bp = np.dot(ab, bp)

        dist = 0

        if (ab_bp > 0):
            dist = np.linalg.norm(np.array(b)-np.array(p))
        elif (ab_ap < 0):
            dist = np.linalg.norm(np.array(a)-np.array(p))
        else:
            x1 = ab[0]
            y1 = ab[1]
            x2 = ap[0]
            y2 = ap[1]
            m = np.sqrt(x1**2 + y1**2)
            dist = np.abs(x1 * x2 - y1 * y2) / m

        return dist
    ################### END_CITATION [1] #################

    def iterate_rrt(self, obstacles):
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

        collides = False
        for o in obstacles:
            tf = o.get_data_transform().transform(q_new.get_vertex())
            if o.contains_point(tf):
                collides = True
                points = tuple(vec*x for x in np.arange(0.001, 1, 0.001))
                for point in reversed(points):
                    new_point = q_near.get_vertex() + point
                    tf = o.get_data_transform().transform(new_point)
                    if not o.contains_point(tf):
                        q_new = Node((q_near.vertex[0] + point[0], q_near.vertex[1] + point[1]))
                        collides = False
                        break
                break
        
        if collides == False:
            self.tree.insert_vertex(q_new)
            self.tree.insert_edges(q_new, q_near)

        for o in obstacles:
            shortest_dist = self.minDist(self.q_goal.get_vertex(), q_new.get_vertex(), o.center)
            if shortest_dist < o.get_radius():
                return False
            
        self.tree.insert_vertex(self.q_goal)
        self.tree.insert_edges(self.q_goal, q_new)
        return True



    def run_rrt(self, obstacles):
        for _ in range(self.K):
            if self.iterate_rrt(obstacles):
                return self.tree
        return self.tree

#[1] "Minimum distance from a point to the line segment using Vectors", GeeksForGeeks, 2024, https://www.geeksforgeeks.org/minimum-distance-from-a-point-to-the-line-segment-using-vectors/