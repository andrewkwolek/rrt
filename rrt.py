from tree import Tree, Node
import numpy as np
import random
from collections import deque

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
            x3 = p[0]
            x2 = b[0]
            x1 = a[0]
            y3 = p[1]
            y2 = b[1]
            y1 = a[1]

            u_top = ((x3 - x1)*(x2 - x1) + (y3 - y1)*(y2 - y1))
            u_bot = np.sqrt(ab[0]**2 + ab[1]**2)**2
            u = u_top / u_bot
            x = x1 + u*(x2 - x1)
            y = y1 + u*(y2 - y1)
            dist = np.linalg.norm(np.array([x, y])-np.array(p))

        return dist
    ################### END_CITATION [1] #################

    def iterate_rrt(self, obstacles):
        # Generate random vertex q_rand
        q_rand = Node((random.randrange(self.D), random.randrange(self.D)))
        min_dist = 99999
        q_near = None

        # Find the node in the tree that is closest to q_rand
        for v in self.tree.get_vertices():
            dist = np.linalg.norm(np.array(q_rand.vertex) - np.array(v.vertex))
            if dist < min_dist:
                q_near = v
                min_dist = dist
        
        # Find the unit vector from q_near to q_rand and create a new node q_new that is one delta
        # from q_near in the direction of q_rand
        vec = (np.array(q_rand.vertex) - np.array(q_near.vertex)) / min_dist * self.delta
        q_new = Node((q_near.vertex[0] + vec[0], q_near.vertex[1] + vec[1]))

        # Iterate through all of the generated obstacles and determine if q_new is in collision
        # with an obstacle. If it is, find the closest point along the path that does not 
        # collide with the obstacle and set it as q_new.
        collides = False
        for o in obstacles.get_obs():
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
        
        # If we have found a q_new that does not collide with an obstacle, add it to the tree
        if collides == False:
            self.tree.insert_vertex(q_new)
            self.tree.insert_edges(q_new, q_near)

        # Check if there is an unobstructed path from q_new to q_goal. If not, return false to move to next iteration
        for o in obstacles.get_obs():
            shortest_dist = self.minDist(self.q_goal.get_vertex(), q_new.get_vertex(), o.center)
            if shortest_dist < o.get_radius():
                return False
        
        # Return true if there is an unobstructed path from q_new to q_goal and add q_goal to the tree
        self.tree.insert_vertex(self.q_goal)
        self.tree.insert_edges(self.q_goal, q_new)
        return True

    def trace_path(self):
        q = self.q_goal
        path = []
        while q.parent != None:
            path.append(q.get_vertex())
            q.set_path()
            q = q.parent
        return path
            
    def plot(self, axes):
        q = deque()
        q.append(self.q_init)
        axes.scatter(self.q_init.get_vertex()[0], self.q_init.get_vertex()[1], s=2, color="red")
        while q:
            node = q.popleft()
            for child in node.edges:
                if child.is_path:
                    axes.scatter(child.vertex[0], child.vertex[1], s=2, color="red")
                    axes.plot([node.vertex[0], child.vertex[0]], [node.vertex[1], child.vertex[1]], linewidth="0.5", color="red")
                else:
                    axes.scatter(child.vertex[0], child.vertex[1], s=2, color="blue")
                    axes.plot([node.vertex[0], child.vertex[0]], [node.vertex[1], child.vertex[1]], linewidth="0.5", color="blue")
                q.append(child)
    

    def run_rrt(self, obstacles):
        for _ in range(self.K):
            if self.iterate_rrt(obstacles):
                return self.tree
        return self.tree

#[1] "Minimum distance from a point to the line segment using Vectors", GeeksForGeeks, 2024, https://www.geeksforgeeks.org/minimum-distance-from-a-point-to-the-line-segment-using-vectors/