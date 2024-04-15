import numpy as np
import matplotlib.pyplot as plt


class Draw3D:
    def __init__(self):
        pass

    @staticmethod
    def draw_sphere(center, radius, num_points):
        u = np.linspace(0, 2 * np.pi, num_points)
        v = np.linspace(0, np.pi, num_points)
        u, v = np.meshgrid(u, v)
        x = center[0] + radius * np.cos(u) * np.sin(v)
        y = center[1] + radius * np.sin(u) * np.sin(v)
        z = center[2] + radius * np.cos(v)
        return x, y, z

    @staticmethod
    def draw_cylinder(center, radius, height, num_points):
        theta = np.linspace(0, 2 * np.pi, num_points)
        z = np.linspace(center[2], center[2] + height, num_points)
        theta, z = np.meshgrid(theta, z)
        x = center[0] + radius * np.cos(theta)
        y = center[1] + radius * np.sin(theta)
        return x, y, z

    @staticmethod
    def draw_cone(center, radius, height, num_points):
        theta = np.linspace(0, 2 * np.pi, num_points)
        z = np.linspace(center[2], center[2] + height, num_points)
        theta, z = np.meshgrid(theta, z)
        x = center[0] + radius * (1 - z / (center[2] + height)) * np.cos(theta)
        y = center[1] + radius * (1 - z / (center[2] + height)) * np.sin(theta)
        return x, y, z


if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    draw3d = Draw3D()

    # 绘制球形
    center = (0, 0, 6)
    radius = 1
    num_points = 6
    x, y, z = draw3d.draw_sphere(center, radius, num_points)
    points = np.column_stack((x.flatten(), y.flatten(), z.flatten()))
    print(points)
    for i in range(len(x)):
        ax.scatter(x[i], y[i], z[i], c="r", marker="^")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.show()
