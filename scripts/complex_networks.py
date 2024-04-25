import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import normalize
from sklearn.decomposition import PCA
import os
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from enum import Enum
import random
from collections import deque

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
# raw data: yts_raw一直累计 yts、noise每200s清除一次
yts_raw = deque(maxlen=10000)
yts = []
noises = []
Ris = []

FIRST_KILL_FLAG = False
FIRST_REWIRE_FLAG = False
FIRST_TOUGHNESS_FLAG = False
KILL_FLAG = False
REWIRE_FLAG = False
TOUGHNESS_FLAG = False


class ThreatMode(Enum):
    PickUavWithMaxLink = 0
    PickUavRandom = 1


class ComplexNetworks:
    def __init__(self, adj_matrix):
        self.adj_matrix = adj_matrix  # 邻接矩阵
        self.graph = nx.from_numpy_array(adj_matrix)
        self.calculateParam()
        self.is_wired = False
        self.nodes_MY = []
        self.threatMode = ThreatMode.PickUavWithMaxLink

    def calculateParam(self):
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
        self.topsis_weights = self.calculateTopsis()

    def updateGraph(self, new_adj_matrix):
        self.adj_matrix = new_adj_matrix
        self.graph = nx.from_numpy_array(new_adj_matrix)
        self.calculateParam()

    def update_graph_by_json(self, json_data):
        # 通过json数据更新图
        pass

    def calculateTopsis(self):
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

    def degreePdf(self):
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

    def getNumberOfAliveUavs(self):
        alive_uav_number = 0
        for node in self.nodes_MY:
            alive_uav_number = alive_uav_number + (not node.is_damaged)
        return alive_uav_number

    def NLinkIDSizeOfUavs(self, taregt_id):
        target_node = self.nodes_MY[taregt_id]

        return len(target_node.wired_nodes)

    # 组网
    def wireDistance(
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
        for i in range(1, NUM):
            if i == 1:
                node = self.nodes_MY[i]
                node.wired_nodes.append(self.nodes_MY[0].ID)
                self.nodes_MY[0].wired_nodes.append(node.get_ID())
            else:
                self.nodes_MY[i].connect(self.nodes_MY)
        self.is_wired = True

        # 重连
        for k in range(10):
            for i in range(1, NUM):
                if i == 1:
                    pass
                else:
                    self.nodes_MY[i].connect(self.nodes_MY)
                # TODO
                connected_nodes = self.nodes_MY[i].wired_nodes
                for j in connected_nodes:
                    global_adj_matrix[i, j] = 1
                    global_adj_matrix[j, i] = 1

        self.updateGraph(global_adj_matrix)
        if DEBUG:
            print("global_adj_matrix: ", global_adj_matrix)

    # 动态距离检测
    def disDetection(self):
        # 动态监视距离
        global sorted_model_states, global_adj_matrix
        for i in range(NUM):
            self.nodes_MY[i].pos = [
                sorted_model_states[i][1].position.x,
                sorted_model_states[i][1].position.y,
                sorted_model_states[i][1].position.z,
            ]
            for ID in self.nodes_MY[i].wired_nodes:
                pos = [
                    sorted_model_states[ID][1].position.x,
                    sorted_model_states[ID][1].position.y,
                    sorted_model_states[ID][1].position.z,
                ]
                dis = calculateDistance(self.nodes_MY[i].pos, pos)
                if dis > WireVar.rc:
                    self.nodes_MY[i].wired_nodes.remove(ID)
                    global_adj_matrix[i][ID] = 0
                    global_adj_matrix[ID][i] = 0
            if len(self.nodes_MY[i].wired_nodes) == 0:
                if not self.nodes_MY[i].is_damaged:
                    self.nodes_MY[i].connect(self.nodes_MY)
                    connected_nodes = self.nodes_MY[i].wired_nodes
                    for j in connected_nodes:
                        global_adj_matrix[i, j] = 1
                        global_adj_matrix[j, i] = 1

        # 消息生成

    def msgGenerator(self, event):
        # 消息生成
        for i in range(NUM):
            if self.getNumberOfAliveUavs() > 0:
                prob = random.random()
                if prob > (1 - msgGeneratorProb):
                    target_id = random.randint(0, NUM - 1)
                    while self.adj_matrix[i][target_id] == 0:
                        target_id = random.randint(0, NUM - 1)
                    try:
                        shortest_path = nx.shortest_path(
                            self.graph, source=i, target=target_id
                        )
                        uav_msg = UAVMsg(shortest_path)
                        self.nodes_MY[i].msgs.append(uav_msg)
                    except:
                        pass

        # 消息的传递
        yt_raw = 0
        # TODO 增加消息传递step数据
        for i in range(NUM):
            msg_to_remove = []
            for msg in self.nodes_MY[i].msgs:
                if msg.nowID == msg.targetID:
                    yt_raw = yt_raw + 1
                    msg.removeFlag = True
                    msg_to_remove.append(msg)
                else:
                    # TODO fix a bug source需要用msg.nowID
                    msg.updateNowID()
                    shortest_path = nx.shortest_path(
                        self.graph, source=msg.nowID, target=msg.targetID
                    )
                    msg.updateShortPath(shortest_path)
            for msg_remove in msg_to_remove:
                self.nodes_MY[i].msgs.remove(msg_remove)

        # 计算y(t)并保存数据
        global yts_raw, yts, noises
        yts_raw.append(yt_raw)
        with open("../data/yts_raw.txt", "a") as file:
            file.write(str(yt_raw) + "\n")
        if len(yts_raw) > (2 * M_WINDOW + 1):
            sum = 0
            for i in range(2 * M_WINDOW + 1):
                sum = sum + yts_raw[len(yts_raw) - i - 1]
            yt = sum / (2 * M_WINDOW + 1)
            yts.append(yt)
            with open("../data/yts.txt", "a") as file:
                file.write(str(yt) + "\n")
            # 2 * M_WINDOW - 1中点yt_raw值
            noise = yts_raw[len(yts_raw) - M_WINDOW - 1] - yt
            noises.append(noise)

    # 毁伤后节点重连
    def rewire(self, event):
        # print("rewire: ", rospy.Time.now().to_sec())
        global FIRST_REWIRE_FLAG
        FIRST_REWIRE_FLAG = True
        global sorted_model_states, global_adj_matrix
        # global_adj_matrix = np.zeros((NUM, NUM), dtype=int)
        # 重连
        for i in range(1, NUM):
            if i == 1:
                pass
            else:
                if not self.nodes_MY[i].is_damaged:
                    self.nodes_MY[i].connect(self.nodes_MY)
            connected_nodes = self.nodes_MY[i].wired_nodes
            for j in connected_nodes:
                # TODO adj_matrix增加边
                global_adj_matrix[i, j] = 1
                global_adj_matrix[j, i] = 1
        if DEBUG:
            print("global_adj_matrix: ", global_adj_matrix)
        # TODO
        self.updateGraph(global_adj_matrix)

    # 打击毁伤功能
    def chooseTargeUavAndKill(self, event):
        # print("kill: ", rospy.Time.now().to_sec())
        global FIRST_KILL_FLAG
        FIRST_KILL_FLAG = True
        # 毁伤打击
        if self.threatMode == ThreatMode.PickUavWithMaxLink:
            max_degree_node = max(self.graph.degree, key=lambda x: x[1])[0]
            # print("节点： ", max_degree_node)
            self.nodes_MY[max_degree_node].is_damaged = True
            self.nodes_MY[max_degree_node].wired_nodes = []
            for i in range(NUM):
                self.adj_matrix[max_degree_node][i] = 0
                self.adj_matrix[i][max_degree_node] = 0
        else:
            random_node = random.randint(0, NUM - 1)
            self.nodes_MY[random_node].is_damaged = True
            self.nodes_MY[random_node].wired_nodes = []
            for i in range(NUM):
                self.adj_matrix[random_node][i] = 0
                self.adj_matrix[i][random_node] = 0

    # 韧性评估
    def calculateToughness(self, event):
        # print("calculateToughness: ", rospy.Time.now().to_sec())
        global FIRST_TOUGHNESS_FLAG
        FIRST_TOUGHNESS_FLAG = True
        # 计算R_total，数据统计
        global yts_raw, yts, noises, Ris
        Ps = 0
        Pn = 0
        sum_yt = 0
        sum_yD = 0
        sum_ymin = 0
        sum_yR = 0
        R_total = 0
        for i in range(len(yts)):
            temp = yts[i]
            sum_yt = sum_yt + temp
            if i < intervalOfRemove * 0.1 * 0.2:
                sum_yD = sum_yD + temp
            if (i > intervalOfRemove * 0.1 * 0.3) & (i < intervalOfRemove * 0.1 * 0.7):
                sum_ymin = sum_ymin + temp
            if (i > intervalOfRemove * 0.1 * 0.73) & (
                i < intervalOfRemove * 0.1 * 0.98
            ):
                sum_yR = sum_yR + temp
            Ps = Ps + temp * temp
            Pn = Pn + noises[i] * noises[i]

        SNRdB = 10 * np.log10(Ps / Pn)
        yD = sum_yD / intervalOfRemove / 0.2
        ymin = sum_ymin / intervalOfRemove / 0.4
        yR = sum_yR / intervalOfRemove / 0.25
        sigma = sum_yt / yD / len(yts)
        delta = ymin / yD
        tau = (intervalOfRemove + 4) / 2 / len(yts)
        rou = yR / yD
        zeta = 1 / (1 + np.exp(-0.25 * (SNRdB - 15)))
        Ri = 0
        if rou < delta:
            Ri = sigma * rou * (delta + zeta)
        else:
            Ri = sigma * rou * (delta + zeta + 1 - np.power(tau, rou - delta))
        print("SNRdB:", SNRdB)
        print("yD:", yD)
        print("ymin:", ymin)
        print("yR:", yR)
        print("sigma:", sigma)
        print("delta:", delta)
        print("tau:", tau)
        print("rou:", rou)
        print("zeta:", zeta)
        print("Ri:", Ri)

        Ris.append(Ri)
        with open("../data/Ri.txt", "a") as file:
            file.write(str(Ri) + "\n")
        yts = []
        noises = []

        if len(Ris) == 5:
            alpha = 0.06
            sum_wi = 0
            for i in range(len(Ris)):
                sum_wi = sum_wi + np.power(1 - alpha, len(Ris) - i)
            for i in range(len(Ris)):
                wi = np.power(1 - alpha, len(Ris) - i)
                R_total = Ris[i] * wi / sum_wi
                print("R_total: ", R_total)
            with open("../data/R_total.txt", "a") as file:
                file.write(str(R_total) + "\n")


def calculateDistance(pose1, pose2):
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
def updateGraphCallback(msg):
    global global_node_name, sorted_model_states
    global_node_name = sorted(msg.name[1:], key=lambda x: eval(x.split("_")[1]))
    model_states = list(zip(msg.name, msg.pose, msg.twist))[1:]
    sorted_model_states = sorted(model_states, key=lambda x: eval(x[0].split("_")[1]))

    # 进行联网
    if not complex_networks.is_wired:
        complex_networks.wireDistance(1)
    complex_networks.disDetection()


if __name__ == "__main__":
    complex_networks = ComplexNetworks(global_adj_matrix)
    with open("../data/yts_raw.txt", "w") as file:
        pass
    with open("../data/yts.txt", "w") as file:
        pass
    with open("../data/Ri.txt", "w") as file:
        pass
    with open("../data/R_total.txt", "w") as file:
        pass
    with open("../data/average_steps.txt", "w") as file:
        pass
    with open("../data/max_steps.txt", "w") as file:
        pass

    rospy.init_node("pose_subscriber")
    rospy.Subscriber(
        "gazebo/model_states", ModelStates, updateGraphCallback, queue_size=1
    )
    rospy.sleep(rospy.Duration(1))
    timer_msg_generator = rospy.Timer(
        rospy.Duration(0.1), complex_networks.msgGenerator
    )

    rospy.sleep(rospy.Duration(3))
    timer_threat = rospy.Timer(
        rospy.Duration(5), complex_networks.chooseTargeUavAndKill
    )
    timer_rewire = rospy.Timer(
        rospy.Duration(5 + intervalOfRemove * 0.1 / 2), complex_networks.rewire
    )
    timer_toughness = rospy.Timer(
        rospy.Duration(5 + intervalOfRemove * 0.1 * 3 / 4 + 2 * M_WINDOW * 0.1),
        complex_networks.calculateToughness,
    )

    rate = rospy.Rate(10)
    fig, ax = plt.subplots()
    while not rospy.is_shutdown():
        complex_networks.updateGraph(global_adj_matrix)
        if FIRST_KILL_FLAG == True:
            if KILL_FLAG == False:
                KILL_FLAG = True
                timer_threat.shutdown()
                timer_threat_new = rospy.Timer(
                    rospy.Duration(20), complex_networks.chooseTargeUavAndKill
                )
        if FIRST_REWIRE_FLAG == True:
            if REWIRE_FLAG == False:
                REWIRE_FLAG = True
                timer_rewire.shutdown()
                timer_rewire_new = rospy.Timer(
                    rospy.Duration(20), complex_networks.rewire
                )

        if FIRST_TOUGHNESS_FLAG == True:
            if TOUGHNESS_FLAG == False:
                TOUGHNESS_FLAG = True
                timer_toughness.shutdown()
                timer_toughness_new = rospy.Timer(
                    rospy.Duration(20),
                    complex_networks.calculateToughness,
                )
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
            # 保存数据
            with open("../data/yts_raw.txt", "a") as file:
                file.close()
            with open("../data/yts.txt", "a") as file:
                file.close()
            with open("../data/Ri.txt", "a") as file:
                file.close()
            with open("../data/R_total.txt", "a") as file:
                file.close()
            with open("../data/average_steps.txt", "a") as file:
                file.close()
            with open("../data/max_steps.txt", "a") as file:
                file.close()
            continue
