import numpy as np


class GridMap:
    def __init__(self, width=80, height=60):
        self.width = width
        self.height = height
        self.occ = np.zeros((height, width), dtype=np.uint8)

    def add_rect(self, x0, y0, x1, y1):
        self.occ[y0:y1+1, x0:x1+1] = 1

    def is_free(self, x, y):
        ix, iy = int(round(x)), int(round(y))
        return (0 <= ix < self.width and 0 <= iy < self.height
                and self.occ[iy, ix] == 0)

    def neighbors8(self, node):
        x, y = node
        out = []
        for dx in (-1,0,1):
            for dy in (-1,0,1):
                if dx == 0 and dy == 0:
                    continue
                q = (x+dx, y+dy)
                if self.is_free(*q):
                    out.append(q)
        return out

    @staticmethod
    def demo():
        g = GridMap()
        g.add_rect(20, 5, 26, 38)
        g.add_rect(38, 22, 45, 55)
        g.add_rect(55, 5, 61, 37)
        return g
