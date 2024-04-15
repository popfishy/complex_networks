import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

formation_dict_18 = {
    "origin": np.array(
        [
            [3, 0, 0],
            [6, 0, 0],
            [0, 3, 0],
            [3, 3, 0],
            [6, 3, 0],
            [0, 6, 0],
            [3, 6, 0],
            [6, 6, 0],
            [0, 9, 0],
            [3, 9, 0],
            [6, 9, 0],
            [0, 12, 0],
            [3, 12, 0],
            [6, 12, 0],
            [0, 15, 0],
            [3, 15, 0],
            [6, 15, 0],
        ]
    ),
}

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# 绘制形状的点
coords = formation_dict_18["origin"]
x = coords[:, 0]
y = coords[:, 1]
z = coords[:, 2]
ax.scatter(x, y, z, label="origin")

# 标记坐标轴
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# 显示图例
ax.legend()

# 显示图形
plt.show()
