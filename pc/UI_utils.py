class Line(object):
    def __init__(self, id, x1, y1, x2, y2):
        self.id = id; self.x1 = x1; self.y1 = y1; self.x2 = x2; self.y2 = y2

    def __str__(self) -> str:
        return f"Line: {self.id}. ({self.x1}, {self.y1}) - ({self.x2}, {self.y2})"

class Path(object):
    def __init__(self):
        self.lines: list[Line] = []

    def add(self, l: Line):
        self.lines.append(l)

    def extract_points(self) -> list[tuple[float, float]]:
        points = []
        for line in self.lines:
            points.append((line.x1, line.y1))
        last_pt = (self.lines[-1].x2, self.lines[-1].y2)
        points.append(last_pt)
        return points

    def __str__(self) -> str:
        res = "Path containing\n"
        for l in self.lines:
            res = res + str(l) + "\n"
        return res