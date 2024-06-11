import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import normalize
from sklearn.decomposition import PCA
import os
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PyQt5.QtWidgets import QApplication
import sys
import random
from typing import List

from uav import UAVNode
from globals import *
from uav_msg import UAVMsg
from control.msg import Toughness
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
import threading

# global variable
DEBUG = False
global_node_name = [" "] * NUM
global_adj_matrix = np.zeros((NUM, NUM), dtype=int)
sorted_model_states = []
# raw data: yts_raw一直累计 yts、noise每200s清除一次
yts_raw = []
yts = []
noises = []
Ris = []
clear_msgs = 0
start_time = rospy.Time()

# TODO 对照实验： 1.通信距离rc  2.集群规模N   多次实验 实验次数：times
times = 1
file_name = (
    "_rc:" + str(WireVar.rc) + "_N:" + str(NUM) + "_times:" + str(times) + ".txt"
)

FIRST_KILL_FLAG = False
FIRST_REWIRE_FLAG = False
FIRST_TOUGHNESS_FLAG = False
KILL_FLAG = False
REWIRE_FLAG = False
TOUGHNESS_FLAG = False


class ComplexNetworks:
    def __init__(self, adj_matrix):
        self.graph_lock = threading.Lock()
        self.adj_matrix = adj_matrix  # 邻接矩阵
        self.graph = nx.from_numpy_array(adj_matrix)
        self.calculateParam()
        self.is_wired = False
        self.nodes_MY: List[UAVNode] = []
        self.threat_mode = ThreatMode.PickUavWithMaxLink
        self.calculate_toughness_mode = calculateToughnessMode.ComplexNetworksMode

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
        # 先获取互斥锁
        with self.graph_lock:
            self.adj_matrix = new_adj_matrix
            self.graph = nx.from_numpy_array(new_adj_matrix)
            self.calculateParam()
        # TODO 更新nodes_MY中wired_nodes的损毁的边  已经在chooseTargeUavAndKill函数中添加
        # for i in range(NUM):
        #     if self.nodes_MY[i].is_damaged:
        #         for j in range(NUM):
        #             self.nodes_MY[j].wired_nodes.remove(i)

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
        # for i in range(1, NUM):
        #     # TODO 仅当nodes_MY[0]和nodes_MY[1]靠近时，能够直接连接两者所有节点，当不靠近时，需要增加距离判断
        #     if i == 1:
        #         pass
        #         # node = self.nodes_MY[i]
        #         # node.wired_nodes.append(self.nodes_MY[0].ID)
        #         # self.nodes_MY[0].wired_nodes.append(node.get_ID())
        #     else:
        #         self.nodes_MY[i].connect(self.nodes_MY)

        # 重连
        for k in range(10):
            for i in range(1, NUM):
                # if i == 1:
                #     pass
                # else:
                self.nodes_MY[i].connect(self.nodes_MY)
                connected_nodes = self.nodes_MY[i].wired_nodes
                for j in connected_nodes:
                    global_adj_matrix[i, j] = 1
                    global_adj_matrix[j, i] = 1
        self.is_wired = True

        self.updateGraph(global_adj_matrix)
        if DEBUG:
            print("wireDistance之后global_adj_matrix: ", global_adj_matrix)

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
                    # print("节点:", i, "重新连接的:", connected_nodes)
                    for j in connected_nodes:
                        global_adj_matrix[i, j] = 1
                        global_adj_matrix[j, i] = 1
        self.updateGraph(global_adj_matrix)

    def msgGenerator(self, event):
        # 消息生成
        for i in range(NUM):
            if (self.getNumberOfAliveUavs() > 0) & (not self.nodes_MY[i].is_damaged):
                prob = random.random()
                if prob > (1 - msgGeneratorProb):
                    target_id = random.randint(0, NUM - 1)
                    while i == target_id:
                        target_id = random.randint(0, NUM - 1)
                    # 线程锁，防止冲突
                    with self.graph_lock:
                        temp_graph = self.graph
                    # if nx.has_path(temp_graph, i, target_id):
                    #     shortest_path = nx.shortest_path(
                    #         temp_graph, source=i, target=target_id
                    #     )
                    #     uav_msg = UAVMsg(shortest_path)
                    #     self.nodes_MY[i].msgs.append(uav_msg)
                    # else:
                    #     uav_msg = UAVMsg()
                    #     uav_msg.soureID = i
                    #     uav_msg.nowID = i
                    #     uav_msg.targetID = -1
                    #     uav_msg.shortPath = [uav_msg.soureID, uav_msg.targetID]
                    #     self.nodes_MY[i].msgs.append(uav_msg)
                    try:
                        shortest_path = nx.shortest_path(
                            temp_graph, source=i, target=target_id
                        )
                        uav_msg = UAVMsg(shortest_path)
                        self.nodes_MY[i].msgs.append(uav_msg)
                    except nx.NetworkXNoPath:
                        uav_msg = UAVMsg()
                        uav_msg.soureID = i
                        uav_msg.nowID = i
                        uav_msg.targetID = -1
                        uav_msg.shortPath = [uav_msg.soureID, uav_msg.targetID]
                        self.nodes_MY[i].msgs.append(uav_msg)

        global yts_raw, yts, noises, start_time, clear_msgs, global_adj_matrix
        yt_raw = 0
        yt = 0

        # TODO 增加消息传递step数据
        for i in range(NUM):
            msg_to_remove = []
            for msg in self.nodes_MY[i].msgs:
                if msg.nowID == msg.targetID:
                    yt_raw = yt_raw + 1
                    msg.removeFlag = True
                    msg_to_remove.append(msg)
                elif msg.targetID == -1:
                    clear_msgs = clear_msgs + 1
                    msg.removeFlag = True
                    msg_to_remove.append(msg)
                else:
                    # fix a bug source需要用msg.nowID
                    msg.updateNowID()
                    try:
                        shortest_path = nx.shortest_path(
                            self.graph, source=msg.nowID, target=msg.targetID
                        )
                        msg.updateShortPath(shortest_path)
                    except nx.NetworkXNoPath:
                        msg.removeFlag = True
                        msg_to_remove.append(msg)

            for msg_remove in msg_to_remove:
                self.nodes_MY[i].msgs.remove(msg_remove)

        # 计算y(t)并保存数据
        yts_raw.append(yt_raw)
        with open("../data/yts_raw" + file_name, "a") as file:
            file.write(str(yt_raw) + "\n")
        if len(yts_raw) > (2 * M_WINDOW + 1):
            sum = 0
            for i in range(2 * M_WINDOW + 1):
                sum = sum + yts_raw[len(yts_raw) - i - 1]
            yt = sum / (2 * M_WINDOW + 1)
            yts.append(yt)
            with open("../data/yts" + file_name, "a") as file:
                file.write(str(yt) + "\n")
            # 2 * M_WINDOW - 1中点yt_raw值
            noise = yts_raw[len(yts_raw) - M_WINDOW - 1] - yt
            noises.append(noise)

        # for visualization
        global start_time
        msg_generator_time = rospy.Time.now().to_sec()
        toughness_msg = Toughness()
        toughness_msg.matrix = np.ravel(global_adj_matrix)
        toughness_msg.rows, toughness_msg.cols = global_adj_matrix.shape
        toughness_msg.time = msg_generator_time - start_time
        toughness_msg.yt = yt
        toughness_msg.yt_raw = yt_raw
        toughness_msg.clear_msgs = clear_msgs

        pub.publish(toughness_msg)

    # 毁伤后节点重连
    def rewire(self, event):
        # print("rewire: ", rospy.Time.now().to_sec())
        global FIRST_REWIRE_FLAG
        FIRST_REWIRE_FLAG = True
        global sorted_model_states, global_adj_matrix
        # global_adj_matrix = np.zeros((NUM, NUM), dtype=int)
        # 重连
        for i in range(NUM):
            self.nodes_MY[i].connect(self.nodes_MY)
            for j in self.nodes_MY[i].wired_nodes:
                global_adj_matrix[i, j] = 1
                global_adj_matrix[j, i] = 1
        if DEBUG:
            print("rewire之后global_adj_matrix: ", global_adj_matrix)
        self.updateGraph(global_adj_matrix)

    # 打击毁伤功能
    def chooseTargeUavAndKill(self, event):
        # print("毁伤: ", rospy.Time.now().to_sec())
        global global_adj_matrix
        global FIRST_KILL_FLAG
        FIRST_KILL_FLAG = True
        # 毁伤打击
        sum = 0
        if self.threat_mode == ThreatMode.PickUavWithMaxLink:
            max_degree_node = max(self.graph.degree, key=lambda x: x[1])[0]
            print("选择打击节点： ", max_degree_node)
            self.nodes_MY[max_degree_node].is_damaged = True
            self.nodes_MY[max_degree_node].wired_nodes = []
            for i in range(NUM):
                sum = sum + self.nodes_MY[i].is_damaged
                global_adj_matrix[max_degree_node][i] = 0
                global_adj_matrix[i][max_degree_node] = 0
                if max_degree_node in self.nodes_MY[i].wired_nodes:
                    self.nodes_MY[i].wired_nodes.remove(max_degree_node)
        else:
            random_node = random.randint(0, NUM - 1)
            self.nodes_MY[random_node].is_damaged = True
            self.nodes_MY[random_node].wired_nodes = []
            for i in range(NUM):
                global_adj_matrix[random_node][i] = 0
                global_adj_matrix[i][random_node] = 0
        self.updateGraph(global_adj_matrix)
        print("打击完成，且损毁节点的数量为: ", sum)

    # 韧性评估
    def calculateToughness(self, event):
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
        # 对第一次的yts长度进行处理
        yts = yts[-intervalOfRemove:]
        noises = noises[-intervalOfRemove:]
        for i in range(min(len(yts), len(noises))):
            temp = yts[i]
            sum_yt = sum_yt + temp
            if i < intervalOfRemove * 0.2:
                sum_yD = sum_yD + temp
            if (i > intervalOfRemove * 0.3) & (i < intervalOfRemove * 0.7):
                sum_ymin = sum_ymin + temp
            if (i > intervalOfRemove * 0.73) & (i < intervalOfRemove * 0.98):
                sum_yR = sum_yR + temp
            Ps = Ps + temp * temp
            Pn = Pn + noises[i] * noises[i]

        # 基于复杂网络计算yD
        if self.calculate_toughness_mode == calculateToughnessMode.ComplexNetworksMode:
            yD = sum_yD / intervalOfRemove / 0.2
        # 基于信息交互计算yD
        elif (
            self.calculate_toughness_mode
            == calculateToughnessMode.InformationExchangeMode
        ):
            yD = msgGeneratorProb * NUM

        SNRdB = 10 * np.log10(Ps / Pn)
        ymin = sum_ymin / intervalOfRemove / 0.4
        yR = sum_yR / intervalOfRemove / 0.25
        sigma = sum_yt / yD / min(len(yts), len(noises))
        delta = ymin / yD
        rou = yR / yD
        tau = (intervalOfRemove + 4) / 2 / min(len(yts), len(noises))
        zeta = 1 / (1 + np.exp(-0.25 * (SNRdB - 15)))
        Ri = 0
        if rou < delta:
            Ri = sigma * rou * (delta + zeta)
        else:
            Ri = sigma * rou * (delta + zeta + 1 - np.power(tau, rou - delta))
        print("----------------------------------start--------------------------------")
        print("yts长度: ", len(yts))
        print("noises长度:", len(noises))
        print("Ps:", Ps)
        print("Pn:", Pn)
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
        print("----------------------------------end--------------------------------")

        Ris.append(Ri)
        with open("../data/Ri" + file_name, "a") as file:
            file.write(str(Ri) + "\n")
        yts.clear()
        noises.clear()
        # yts_raw = yts_raw[-int(intervalOfRemove / 2) :]

        if len(Ris) >= 5:
            Ris_temp = Ris[-5:]
            alpha = 0.06
            sum_wi = 0
            for i in range(len(Ris_temp)):
                sum_wi = sum_wi + np.power(1 - alpha, len(Ris_temp) - 1 - i)
            for i in range(len(Ris_temp)):
                wi = np.power(1 - alpha, len(Ris_temp) - 1 - i)
                R_total = R_total + Ris_temp[i] * wi / sum_wi
            print("R_total: ", R_total)
            with open("../data/R_total" + file_name, "a") as file:
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
    # TODO 对照实验： 1.通信距离rc  2.集群规模N   多次实验 实验次数：times
    times = 0
    with open("../data/yts_raw" + file_name, "w") as file:
        pass
    with open("../data/yts" + file_name, "w") as file:
        pass
    with open("../data/Ri" + file_name, "w") as file:
        pass
    with open("../data/R_total" + file_name, "w") as file:
        pass
    with open("../data/average_steps" + file_name, "w") as file:
        pass
    with open("../data/max_steps" + file_name, "w") as file:
        pass

    rospy.init_node("complex_networks")
    rospy.Subscriber(
        "gazebo/model_states", ModelStates, updateGraphCallback, queue_size=1
    )
    pub = rospy.Publisher("ui/toughness", Toughness, queue_size=1)

    rospy.sleep(rospy.Duration(1.0))
    start_time = rospy.Time.now().to_sec()
    timer_msg_generator = rospy.Timer(
        rospy.Duration(0.1), complex_networks.msgGenerator
    )

    rospy.sleep(rospy.Duration(5))

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

    while not rospy.is_shutdown():
        # complex_networks.updateGraph(global_adj_matrix)
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

        try:
            rate.sleep()
        except:
            # 保存数据
            with open("../data/yts_raw" + file_name, "a") as file:
                file.close()
            with open("../data/yts" + file_name, "a") as file:
                file.close()
            with open("../data/Ri" + file_name, "a") as file:
                file.close()
            with open("../data/R_total" + file_name, "a") as file:
                file.close()
            with open("../data/average_steps" + file_name, "a") as file:
                file.close()
            with open("../data/max_steps" + file_name, "a") as file:
                file.close()
