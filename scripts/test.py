# 导入库
import networkx as nx
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

matrix = np.array(
    [
        [0, 0, 0, 0, 0],
        [0, 0, 1, 1, 1],
        [0, 1, 0, 0, 0],
        [0, 1, 0, 0, 1],
        [0, 1, 0, 1, 0],
    ]
)
G = nx.from_numpy_array(matrix)
# 邻接矩阵
As = nx.adjacency_matrix(G)
A = As.todense()
print(A)

# 两个节点间的一条最短路径
shortest_path = nx.shortest_path(G, source=2, target=1)
print("两个节点间的一条最短路径: ", shortest_path)
print(np.sum(matrix))
nx.draw(G, node_size=1000, with_labels=True)
plt.show()
