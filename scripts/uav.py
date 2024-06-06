import cv2
import random
import numpy as np
from globals import WireVar
from collections import deque
from typing import List


class UAVNode:
    def __init__(self, ID):
        self.ID: int = ID
        self.pos = [0, 0, 0]
        self.wired_nodes: List[int] = []
        self.msgs = deque(maxlen=200)
        # TODO have not used
        self.is_damaged = False

    def __init__(self, ID, pose):
        self.ID: int = ID
        self.pos = [pose.position.x, pose.position.y, pose.position.z]
        self.wired_nodes: List[int] = []
        self.msgs = deque(maxlen=200)
        self.is_damaged = False
        # self.MaxSenceProbability = 0.9

    def all_connect(self, nodes):
        if len(nodes) == 0:
            pass
        else:
            for node in nodes:
                self.wire_node(node)

    # 考虑地理距离的无标度网络连接方式
    # reference: 《基于信息交互的无人机集群建模与韧性评估》
    def connect(self, nodes):
        # 参考p25公式：rc为通信距离
        rc = WireVar.rc
        e = WireVar.e
        alpha = WireVar.alpha

        sum_k = 0.0
        for node in nodes:
            if (
                node.ID != self.ID
                and self.ID not in node.wired_nodes
                and self.get_distance(node) < rc
            ):
                d = self.get_distance(node)
                if d < alpha * rc:
                    sum_k += len(node.wired_nodes) + e
                else:
                    sum_k += (len(node.wired_nodes) + e) * (rc - d) / (rc - alpha * rc)
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
                    if d < alpha * rc:
                        count += len(node.wired_nodes) + e
                    else:
                        count += (
                            (len(node.wired_nodes) + e) * (rc - d) / (rc - alpha * rc)
                        )

                if count > chose:
                    self.wire_node(node)
                    break

    def re_connect(self, nodes):
        # 参考p25公式：rc为通信距离
        rc = WireVar.rc
        e = WireVar.e
        alpha = WireVar.alpha

        sum_k = 0.0
        for node in nodes:
            if (
                node.ID != self.ID
                and self.ID not in node.wired_nodes
                and self.get_distance(node) < rc
                and (not node.is_damaged)
            ):
                d = self.get_distance(node)
                if d < alpha * rc:
                    sum_k += len(node.wired_nodes) + e
                else:
                    sum_k += (len(node.wired_nodes) + e) * (rc - d) / (rc - alpha * rc)
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
                    and (not node.is_damaged)
                ):
                    d = self.get_distance(node)
                    if d < alpha * rc:
                        count += len(node.wired_nodes) + e
                    else:
                        count += (
                            (len(node.wired_nodes) + e) * (rc - d) / (rc - alpha * rc)
                        )

                if count > chose:
                    self.wire_node(node)
                    break
        return nodes

    def wire_node(self, node):
        if (not self.is_damaged) and (not node.is_damaged):
            if not self.ID in node.wired_nodes:
                node.wired_nodes.append(self.ID)
            if not node.get_ID() in self.wired_nodes:
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
