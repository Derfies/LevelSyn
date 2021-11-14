from reactor.geometry.vector import Vector3


class LineBase:

    def __init__(self, pos1, pos2):
        self.pos1 = pos1
        self.pos2 = pos2

    @property
    def length(self):
        return (self.pos2 - self.pos1).mag()

    @property
    def sq_length(self):
        return (self.pos2 - self.pos1).mag2()

    @property
    def direction(self):
        return self.pos2 - self.pos1

    @property
    def direction3d(self):
        direction = self.direction
        return Vector3(direction[0], direction[1], 0)

    def translate(self, trans):
        self.pos1 += trans
        self.pos2 += trans
