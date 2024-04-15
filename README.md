## 复杂网络

#### 一、环境

环境：Ubuntu20.04 + ros noetic + gazebo 11 + PX4 1.13版本

参考配置：见[XTDrone使用文档](https://www.yuque.com/xtdrone/manual_cn)

#### 二、代码说明

代码基于**[加速仿真、瞬移飞机与大规模集群](https://www.yuque.com/xtdrone/manual_cn/accelerate_sim_and_large_swarm)**模块搭建大规模无人机集群仿真，在该仿真环境中，停止Gazebo内置的物理引擎，直接控制无人机模型位置真值，进行无人机集群编队任务，研究多节点（40个以上）的复杂网络问题。