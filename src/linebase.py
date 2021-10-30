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
