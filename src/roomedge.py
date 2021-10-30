from reactor.geometry.vector import Vector3
from linebase import LineBase


class RoomEdge(LineBase):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.idx1 = -1
        self.idx2 = -1
        self.door_flag = True

    def __str__(self):
        return f'RoomEdge: {self.pos1} {self.pos2}'

    @property
    def direction(self):
        return self.pos2 - self.pos1

    @property
    def direction3d(self):
        direction = self.direction
        return Vector3(direction[0], direction[1], 0)
