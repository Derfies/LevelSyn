from reactor.geometry.vector import Vector2

class LineBase:

    def __init__(self, *args):
        if len(args) == 1:
            pos1 = args[0]
            pos2 = Vector2(*pos1)
        else:
            pos1, pos2 = args
        self.pos1 = pos1
        self.pos2 = pos2

    @property
    def length(self):
        return (self.pos2 - self.pos1).mag()

    @property
    def sq_length(self):
        return (self.pos2 - self.pos1).mag2()
