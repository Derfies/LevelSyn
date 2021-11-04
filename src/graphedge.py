class GraphEdge:

    def __init__(self, g, edge):
        self.g = g
        self.edge = edge

    @property
    def idx0(self):
        return list(self.g.nodes).index(self.edge[0])

    @property
    def idx1(self):
        return list(self.g.nodes).index(self.edge[1])

    @property
    def indices(self):
        return self.idx0, self.idx1
