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
    
    def insert_edges(self, q1: Node, q2: Node):
        q2.edges.append(q1)
        q1.set_parent(q2)

    def get_vertices(self):
        return self.vertices
