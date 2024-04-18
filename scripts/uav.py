from enum import Enum
import cv2
import random
import numpy as np


class UAVNode:
    def __init__(self, ID):
        self.ID = ID
        self.pos = [0, 0, 0]
        self.wired_nodes = []

    def __init__(self, ID, pose):
        self.ID = ID
        self.pos = [pose.position.x, pose.position.y, pose.position.z]
        self.wired_nodes = []
        # self.MaxSenceProbability = 0.9

    def all_connect(self, nodes):
        if not nodes:
            pass
        else:
            for node in nodes:
                self.wire_node(node)

    # 考虑地理距离的无标度网络连接方式
    # reference: 《基于信息交互的无人机集群建模与韧性评估》
    def connect(self, nodes):
        # 参考p25公式：rc为通信距离
        rc = 5.0
        gama = -5
        beta = 0.8
        e = 0.1
        alfa = 0.8

        sum_k = 0.0
        for node in nodes:
            if (
                node.ID != self.ID
                and self.ID not in node.wired_nodes
                and self.get_distance(node) < rc
            ):
                d = self.get_distance(node)
                if d < alfa * rc:
                    sum_k += len(node.wired_nodes) + e
                else:
                    sum_k += (len(node.wired_nodes) + e) * (rc - d) / (rc - alfa * rc)
        if sum_k == 0.0:
            # print("sum_k == 0, ID =", self.ID)
            pass
        else:
            chose = random.uniform(0, sum_k)
            count = 0.0
            for node in nodes:
                if (
                    node.ID != self.ID
                    and self.ID not in node.wired_nodes
                    and self.get_distance(node) < rc
                ):
                    d = self.get_distance(node)
                    if d < alfa * rc:
                        count += len(node.wired_nodes) + e
                    else:
                        count += (
                            (len(node.wired_nodes) + e) * (rc - d) / (rc - alfa * rc)
                        )

                if count > chose:
                    self.wire_node(node)
                    break

    def wire_node(self, node):
        node.wired_nodes.append(self.ID)
        self.wired_nodes.append(node.get_ID())

    def get_distance(self, node):
        return np.sqrt(
            (self.pos[0] - node.pos[0]) ** 2
            + (self.pos[1] - node.pos[1]) ** 2
            + (self.pos[2] - node.pos[2]) ** 2
        )

    def get_ID(self):
        return self.ID

    def get_pos(self):
        return self.pos
