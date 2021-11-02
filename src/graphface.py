class GraphFace:

    def __init__(self, g, indices):
        self.g = g
        self.indices = indices

    @property
    def size(self):
        return len(self.indices)