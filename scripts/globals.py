from enum import Enum

# variables
# 无人机集群规模
NUM = 100
# 无人机个体每秒产生消息的概率
msgGeneratorProb = 0.5
# 毁伤——自适应恢复事件的周期
intervalOfRemove = 200
# 稳健性计算模型中，方窗滤波的参数
M_WINDOW = 10
numberOfRemovedNodesPerTime = 2


class ThreatMode(Enum):
    PickUavWithMaxLink = 0
    PickUavRandom = 1


class calculateToughnessMode(Enum):
    InformationExchangeMode = 0
    ComplexNetworksMode = 1


# 考虑地理距离的无标度网络连接方式  参考p25公式：rc为通信距离
class WireVar:
    rc = 8.0
    e = 0.1
    alpha = 0.8
