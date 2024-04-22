# 导入库
import networkx as nx
import matplotlib.pyplot as plt

# G = nx.Graph()
# G.add_nodes_from([0, 1, 2, 3, 4])
# G.add_edges_from([(0, 1), (0, 2), (1, 3), (1, 2), (1, 4), (3, 4)])
# # 邻接矩阵
# As = nx.adjacency_matrix(G)
# A = As.todense()

# # 两个节点间的一条最短路径
# shortest_path = nx.shortest_path(G, source=0, target=3)
# print("两个节点间的一条最短路径: ", shortest_path)

# # 两个节点之间所有的最短路径
# all_shortest_path = list(nx.all_shortest_paths(G, source=0, target=3))
# print("两个节点之间所有的最短路径: ", all_shortest_path)

# # 求两个节点的最短路径长度（距离）
# shortest_dis = nx.shortest_path_length(G, source=0, target=3)
# print("两个节点的最短路径长度（距离）: ", shortest_dis)

# nx.draw(G, node_size=1000, with_labels=True)
# plt.show()
for i in range(5):
    print(i)
