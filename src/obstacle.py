class Obstacle(object):
    def __init__(self, xleft, ytop, xright, ybottom):
        self.x1 = xleft
        self.y1 = ytop
        self.x2 = xright
        self.y2 = ybottom

    def center(self):
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)

    def render(self, canvas):
        canvas.create_rectangle(
            self.x1, self.y1, self.x2, self.y2, fill='orange'
        )

    def line_segments(self):
        """
        Decomposes the obstacle into a set of 4 line segments.
        """
        return (
            ((self.x1, self.y1), (self.x2, self.y1)),
            ((self.x1, self.y1), (self.x1, self.y2)),
            ((self.x2, self.y1), (self.x2, self.y2)),
            ((self.x1, self.y2), (self.x2, self.y2)),
        )
