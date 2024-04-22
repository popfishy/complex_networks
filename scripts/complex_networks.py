import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import normalize
from sklearn.decomposition import PCA
import os
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from enum import Enum
import random

from uav import UAVNode
from globals import *
from uav_msg import UAVMsg
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist

# global variable
DEBUG = False
global_node_name = [" "] * NUM
global_adj_matrix = np.zeros((NUM, NUM), dtype=int)
sorted_model_states = []


class ThreatMode(Enum):
    PickUavWithMaxLink = 0
    PickUavRandom = 1


class ComplexNetworks:
    def __init__(self, adj_matrix):
        self.adj_matrix = adj_matrix  # 邻接矩阵
        self.graph = nx.from_numpy_array(adj_matrix)
        self.calculate_param()
        self.is_wired = False
        self.nodes_MY = []
        self.threatMode = ThreatMode.PickUavWithMaxLink

    def calculate_param(self):
        self.degree = nx.degree(self.graph)  # 度
        self.clustering_coefficient = nx.clustering(self.graph)  # 聚类系数
        self.laplacian_matrix = nx.laplacian_matrix(self.graph)  # 拉普拉斯矩阵
        self.degree_centrality = nx.degree_centrality(self.graph)  # 度中心性
        self.betweenness_centrality = nx.betweenness_centrality(
            self.graph
        )  # 介数中心性
        self.closeness_centrality = nx.closeness_centrality(self.graph)  # 紧密中心性
        # self.eigenvector_centrality = nx.eigenvector_centrality(
        #     self.graph
        # )  # 特征向量中心性
        # self.katz_centrality = nx.katz_centrality(self.graph)  # Katz中心性
        self.topsis_weights = self.calculate_topsis()

    def update_graph(self, new_adj_matrix):
        self.adj_matrix = new_adj_matrix
        self.graph = nx.from_numpy_array(new_adj_matrix)
        self.calculate_param()

    def update_graph_by_json(self, json_data):
        # 通过json数据更新图
        pass

    def calculate_topsis(self):
        # topsis算法: 取度中心性、介数中心性、紧密中心性三个指标
        # Step 1: Given original decision matrix and normalize it
        decision_matrix = np.array(
            [
                list(self.degree_centrality.values()),
                list(self.betweenness_centrality.values()),
                list(self.closeness_centrality.values()),
            ]
        ).transpose()

        normalized_matrix = normalize(decision_matrix, norm="max", axis=0)

        # Step 2: Determine weights for each criteria using Principal Component Analysis (PCA)
        pca = PCA(n_components=normalized_matrix.shape[1])
        pca.fit(normalized_matrix)
        weights = pca.explained_variance_ratio_
        weighted_decision_matrix = normalized_matrix * weights

        # Step 3: Determine ideal best and ideal worst solutions and calculate weights of topsis
        ideal_best = np.max(weighted_decision_matrix, axis=0)
        ideal_worst = np.min(weighted_decision_matrix, axis=0)
        topsis_weights = np.linalg.norm(
            weighted_decision_matrix - ideal_worst, axis=1
        ) / (
            np.linalg.norm(weighted_decision_matrix - ideal_best, axis=1)
            + np.linalg.norm(weighted_decision_matrix - ideal_worst, axis=1)
        )
        return topsis_weights

    def degree_pdf(self):
        # 度分布
        x = list(range(max(self.degree.values()) + 1))
        y = [i / len(self.graph.nodes) for i in nx.degree_histogram(self.graph)]
        return x, y

    def visualization(self, ax):
        # 更新边
        lines = ax.lines  # 获取当前图形中的所有线条对象
        for line in lines:
            line.remove()  # 删除所有线条对象
        # Compute node positions on the circle
        num_nodes = self.adj_matrix.shape[0]
        angle = 2 * np.pi / num_nodes
        radius = 0.4  # 圆的半径
        node_positions = [
            (0.5 + radius * np.cos(i * angle), 0.5 + radius * np.sin(i * angle))
            for i in range(num_nodes)
        ]

        # Plot edges based on the adjacency matrix
        for i in range(num_nodes):
            for j in range(i + 1, num_nodes):
                if self.adj_matrix[i][j] == 1:
                    plt.plot(
                        [node_positions[i][0], node_positions[j][0]],
                        [node_positions[i][1], node_positions[j][1]],
                        "black",
                    )
        for i in range(num_nodes):
            image = plt.imread(
                os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), "pic/plane.png"
                )
            )
            imagebox = OffsetImage(image, zoom=0.1)
            ab = AnnotationBbox(imagebox, node_positions[i], frameon=False)
            ax.add_artist(ab)
            # 添加节点名称的文本标签
            text_radius = radius + 0.03  # 节点名称的半径
            text_angle = i * angle  # 节点名称的角度
            text_x = 0.5 + text_radius * np.cos(text_angle)  # 节点名称的x坐标
            text_y = 0.5 + text_radius * np.sin(text_angle)  # 节点名称的y坐标
            ax.annotate(
                global_node_name[i],
                xy=(text_x, text_y),
                xytext=(0, 0),  # 文本标签的偏移量
                textcoords="offset points",
                ha="center",
                va="center",
                fontsize=12,
            )
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)
        ax.set_aspect("equal", adjustable="box")
        ax.axis("off")
        x = [node_positions[i][0] for i in range(num_nodes)]
        y = [node_positions[i][1] for i in range(num_nodes)]
        plt.plot(x, y, "ro")
        plt.draw()
        plt.pause(0.001)

    # TODO 此处进行联网
    def wire_distance(
        self,
        init_sizeof_network,
    ):
        global sorted_model_states, global_adj_matrix
        for i in range(NUM):
            if i < init_sizeof_network:
                node_MY = UAVNode(i, sorted_model_states[i][1])
                node_MY.all_connect(self.nodes_MY)
                self.nodes_MY.append(node_MY)
            else:
                node_MY = UAVNode(i, sorted_model_states[i][1])
                node_MY.connect(self.nodes_MY)
                self.nodes_MY.append(node_MY)
        for i in range(NUM):
            if i == 0 or i == 1:
                pass
            else:
                self.nodes_MY[i].connect(self.nodes_MY)
            connected_nodes = self.nodes_MY[i].wired_nodes
            for j in connected_nodes:
                global_adj_matrix[i, j] = 1
        self.is_wired = True
        if DEBUG:
            print("global_adj_matrix: ", global_adj_matrix)

    # TODO 逻辑错误，对于单个无边节点进行重连，不需要重置global_adj_matrix
    def rewire(self, event):
        global sorted_model_states, global_adj_matrix
        global_adj_matrix = np.zeros((NUM, NUM), dtype=int)
        # 动态监视距离
        # TODO 增加消息生成功能
        for i in range(NUM):
            self.nodes_MY[i].pos = [
                sorted_model_states[i][1].position.x,
                sorted_model_states[i][1].position.y,
                sorted_model_states[i][1].position.z,
            ]
            check_nodes = self.nodes_MY[i].wired_nodes
            for ID in check_nodes:
                # self.nodes_MY[ID].pos
                pos = [
                    sorted_model_states[ID][1].position.x,
                    sorted_model_states[ID][1].position.y,
                    sorted_model_states[ID][1].position.z,
                ]
                dis = calculate_distance(self.nodes_MY[i].pos, pos)
                if dis > WireVar.rc:
                    self.nodes_MY[i].wired_nodes.remove(ID)
        # 重连
        for i in range(NUM):
            if i == 0 or i == 1:
                pass
            else:
                self.nodes_MY[i].connect(self.nodes_MY)
            connected_nodes = self.nodes_MY[i].wired_nodes
            for j in connected_nodes:
                global_adj_matrix[i, j] = 1
        if DEBUG:
            print("global_adj_matrix: ", global_adj_matrix)

    # 打击毁伤功能
    def chooseTargeUavAndKill(self, event):
        timer_rewire = rospy.Timer(
            rospy.Duration(intervalOfRemove * 0.05),
            complex_networks.rewire,
            oneshot=True,
        )
        if self.threatMode == ThreatMode.PickUavWithMaxLink:
            max_degree_node = max(self.graph, key=lambda x: x[1])[0]
            print("节点： ", max_degree_node)
            self.nodes_MY[max_degree_node].is_damaged = True
            self.nodes_MY[max_degree_node].wired_nodes = []
            for i in range(NUM):
                self.adj_matrix[max_degree_node][i] = 0
                self.adj_matrix[i][max_degree_node] = 0
        else:
            random_node = random.randint(0, NUM - 1)
            self.nodes_MY[random_node].is_damaged = True
            self.nodes_MY[random_node].wired_nodes = []
            self.adj_matrix[random_node][i] = 0
            self.adj_matrix[i][random_node] = 0

    # 每0.1s产生一个序列
    def msgGenerator(self, event):
        for i in range(NUM):

            prob = random.random()
            if prob > msgGeneratorProb:
                uav_msg = UAVMsg([i, 5])
                self.nodes_MY[i].msgs.append(uav_msg)


def calculate_distance(pose1, pose2):
    # 计算两个姿态之间的欧氏距离
    # dx = pose1.x - pose2.x
    # dy = pose1.y - pose2.y
    # dz = pose1.z - pose2.z
    dx = pose1[0] - pose2[0]
    dy = pose1[1] - pose2[1]
    dz = pose1[2] - pose2[2]

    distance = np.sqrt(dx**2 + dy**2 + dz**2)
    return distance


# 预设距离阈值
def update_graph_callback(msg):
    global global_node_name, sorted_model_states, global_adj_matrix
    global_node_name = sorted(msg.name[1:], key=lambda x: eval(x.split("_")[1]))
    model_states = list(zip(msg.name, msg.pose, msg.twist))[1:]
    sorted_model_states = sorted(model_states, key=lambda x: eval(x[0].split("_")[1]))

    # 进行联网
    if not complex_networks.is_wired:
        complex_networks.wire_distance(2)


if __name__ == "__main__":
    complex_networks = ComplexNetworks(global_adj_matrix)

    rospy.init_node("pose_subscriber")
    rospy.Subscriber(
        "gazebo/model_states", ModelStates, update_graph_callback, queue_size=1
    )
    timer_threat = rospy.Timer(
        rospy.Duration(intervalOfRemove * 0.1), complex_networks.chooseTargeUavAndKill
    )

    rate = rospy.Rate(10)
    fig, ax = plt.subplots()
    while not rospy.is_shutdown():
        complex_networks.update_graph(global_adj_matrix)
        # x = list(range(0, NUM))
        # plt.figure(1)
        # plt.plot(
        #     x,
        #     list(complex_networks.degree_centrality.values()),
        #     label="Degree Centrality",
        # )
        # plt.plot(
        #     x,
        #     list(complex_networks.betweenness_centrality.values()),
        #     label="Betweenness Centrality",
        # )
        # plt.plot(
        #     x,
        #     list(complex_networks.closeness_centrality.values()),
        #     label="Closeness Centrality",
        # )
        # plt.plot(
        #     x,
        #     list(complex_networks.topsis_weights),
        #     label="TOPSIS Centrality",
        # )
        # plt.legend(fontsize="large")
        # plt.figure(2)
        # nx.draw(complex_networks.graph, node_size=500, with_labels=True)
        # plt.show()

        complex_networks.visualization(ax)
        try:
            rate.sleep()
        except:
            continue
