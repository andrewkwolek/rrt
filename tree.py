import matplotlib.pyplot as plt

class Node():
    def __init__(self, q):
        self.vertex = q
        self.edges = []
        self.parent = None
        self.is_path = False

    def get_vertex(self):
        return self.vertex
    
    def set_parent(self, p):
        self.parent = p

    def set_path(self):
        self.is_path = True

class Tree():
    def __init__(self, q):
        self.vertices = []
        self.edges = []
        self.vertices.append(q)
        self.newest = 0

    def insert_vertex(self, q: Node):
        self.vertices.append(q)
        self.newest = len(self.vertices) - 1
        # plt.scatter(q.vertex[0], q.vertex[1], s=2, color="blue")
    
    def insert_edges(self, q1: Node, q2: Node):
        q2.edges.append(q1)
        q1.set_parent(q2)
        # print(q1.vertex)
        # print(q2.vertex)
        # plt.plot([q1.vertex[0], q2.vertex[0]], [q1.vertex[1], q2.vertex[1]], linewidth="0.5", color="blue", zorder=0)

    def get_vertices(self):
        return self.vertices
