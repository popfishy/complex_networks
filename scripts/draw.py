import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.special import binom

bernstein = lambda n, k, t: binom(n, k) * t**k * (1.0 - t) ** (n - k)


def bezier(points, num=200):
    N = len(points)
    t = np.linspace(0, 1, num=num)
    curve = np.zeros((num, 2))
    for i in range(N):
        curve += np.outer(bernstein(N - 1, i, t), points[i])
    return curve


class Segment:
    def __init__(self, p1, p2, angle1, angle2, **kw):
        self.p1 = p1
        self.p2 = p2
        self.angle1 = angle1
        self.angle2 = angle2
        self.numpoints = kw.get("numpoints", 100)
        r = kw.get("r", 0.3)
        d = np.sqrt(np.sum((self.p2 - self.p1) ** 2))
        self.r = r * d
        self.p = np.zeros((4, 2))
        self.p[0, :] = self.p1[:]
        self.p[3, :] = self.p2[:]
        self.calc_intermediate_points(self.r)

    def calc_intermediate_points(self, r):
        self.p[1, :] = self.p1 + np.array(
            [self.r * np.cos(self.angle1), self.r * np.sin(self.angle1)]
        )
        self.p[2, :] = self.p2 + np.array(
            [self.r * np.cos(self.angle2 + np.pi), self.r * np.sin(self.angle2 + np.pi)]
        )
        self.curve = bezier(self.p, self.numpoints)


class Draw:
    def __init__(self) -> None:
        pass

    @staticmethod
    def draw_circle(center, radius, num_points):
        points = np.empty((num_points, 2))
        angle = 2 * math.pi / num_points
        for i in range(num_points):
            x = center[0] + radius * math.cos(i * angle)
            y = center[1] + radius * math.sin(i * angle)
            points[i] = [x, y]
        return points

    @staticmethod
    def draw_square(center, side_length):
        half_length = side_length / 2
        top_left = (center[0] - half_length, center[1] - half_length)
        top_right = (center[0] + half_length, center[1] - half_length)
        bottom_right = (center[0] + half_length, center[1] + half_length)
        bottom_left = (center[0] - half_length, center[1] + half_length)
        points = np.array([top_left, top_right, bottom_right, bottom_left])
        return points

    @staticmethod
    def draw_rectangle(center, width, height):
        half_width = width / 2
        half_height = height / 2
        top_left = (center[0] - half_width, center[1] - half_height)
        top_right = (center[0] + half_width, center[1] - half_height)
        bottom_right = (center[0] + half_width, center[1] + half_height)
        bottom_left = (center[0] - half_width, center[1] + half_height)
        points = np.array([top_left, top_right, bottom_right, bottom_left])
        return points

    @staticmethod
    def draw_triangle(point1, point2, point3):
        points = np.array([point1, point2, point3])
        return points

    @staticmethod
    def draw_ellipse(center, semimajor_axis, semiminor_axis, num_points):
        points = np.empty((num_points, 2))
        angle = 2 * math.pi / num_points
        for i in range(num_points):
            x = center[0] + semimajor_axis * math.cos(i * angle)
            y = center[1] + semiminor_axis * math.sin(i * angle)
            points[i] = [x, y]
        return points

    # 下面为使用bezier曲线的方法
    def ccw_sort(self, p):
        d = p - np.mean(p, axis=0)
        s = np.arctan2(d[:, 0], d[:, 1])
        return p[np.argsort(s), :]

    def get_curve(self, points, **kw):
        segments = []
        for i in range(len(points) - 1):
            seg = Segment(
                points[i, :2], points[i + 1, :2], points[i, 2], points[i + 1, 2], **kw
            )
            segments.append(seg)
        curve = np.concatenate([s.curve for s in segments])
        return segments, curve

    def get_bezier_curve(self, a, rad=0.3, edgy=0.5):
        """given an array of points *a*, create a curve through
        those points.
        *rad* is a number between 0 and 1 to steer the distance of
              control points.
        *edgy* is a parameter which controls how "edgy" the curve is,
              edgy=0 is smoothest."""
        p = np.arctan(edgy) / np.pi + 0.5
        a = self.ccw_sort(a)
        a = np.append(a, np.atleast_2d(a[0, :]), axis=0)
        d = np.diff(a, axis=0)
        ang = np.arctan2(d[:, 1], d[:, 0])
        f = lambda ang: (ang >= 0) * ang + (ang < 0) * (ang + 2 * np.pi)
        ang = f(ang)
        ang1 = ang
        ang2 = np.roll(ang, 1)
        ang = p * ang1 + (1 - p) * ang2 + (np.abs(ang2 - ang1) > np.pi) * np.pi
        ang = np.append(ang, [ang[0]])
        a = np.append(a, np.atleast_2d(ang).T, axis=1)
        s, points = self.get_curve(a, r=rad, method="var")
        # Clip control points to the unit square
        # a[:, :2] = np.clip(a[:, :2], 0, 1)
        return points

    def get_random_points(self, n=5, scale=0.8, mindst=None, rec=0):
        """create n random points in the unit square, which are *mindst*
        apart, then scale them."""
        mindst = mindst or 0.7 / n
        a = np.random.rand(n, 2)
        d = np.sqrt(np.sum(np.diff(self.ccw_sort(a), axis=0), axis=1) ** 2)
        if np.all(d >= mindst) or rec >= 200:
            return a * scale
        else:
            return self.get_random_points(n=n, scale=scale, mindst=mindst, rec=rec + 1)


if __name__ == "__main__":
    # 示例用法
    Draw = Draw()
    circle_points = Draw.draw_circle((6, 8), 2, 20)
    # plt.plot(circle_points[:, 0], circle_points[:, 1], "ro")
    # points = Draw.get_bezier_curve(circle_points, rad=0.2, edgy=0.05)
    # plt.plot(points[:, 0], points[:, 1])
    # plt.show()

    print("Circle Points:")
    print(circle_points)

    square_points = Draw.get_random_points(n=5, scale=4)
    plt.plot(square_points[:, 0], square_points[:, 1], "ro")
    points = Draw.get_bezier_curve(square_points, rad=0.2, edgy=0.05)
    plt.plot(points[:, 0], points[:, 1])
    plt.show()
    print("Square Points:")
    print(square_points)

    rectangle_points = Draw.draw_rectangle((0, 0), 10, 5)
    print("Rectangle Points:")
    print(rectangle_points)

    triangle_points = Draw.draw_triangle((0, 0), (5, 0), (2.5, 5))
    print("Triangle Points:")
    print(triangle_points)

    ellipse_points = Draw.draw_ellipse((0, 0), 8, 4, 30)
    print("Ellipse Points:")
    print(ellipse_points)
