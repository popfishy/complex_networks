import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist
import sys
from draw import Draw
from draw3d import Draw3D
import numpy as np
import math

# 默认在gazebo中大小200x200地图范围内运动
MODE = ["circle", "sphere", "random"]


def pose_publisher(mode):
    vehicle_type = sys.argv[1]
    row_num = int(sys.argv[2])
    column_num = int(sys.argv[3])
    vehicle_num = row_num * column_num
    pub = rospy.Publisher("gazebo/set_model_states", ModelStates, queue_size=1)
    poses_msg = ModelStates()
    draw = Draw()
    draw3d = Draw3D()
    poses_msg.name = [None] * vehicle_num
    poses_msg.pose = [Pose() for i in range(vehicle_num)]
    poses_msg.twist = [Twist() for i in range(vehicle_num)]
    if mode == "circle":
        center_list = [
            np.array([-6, 6]),
            np.array([6, 6]),
            np.array([18, 6]),
            np.array([18, -6]),
            np.array([6, -6]),
            np.array([-6, -6]),
        ]
        points_list = [draw.draw_circle(center, 2, 600) for center in center_list]
        cnt = min([len(points) for points in points_list])
    elif mode == "sphere":
        x, y, z = draw3d.draw_sphere(center=(0, 0, 6), radius=4, num_points=6)
        center_list = np.column_stack((x.flatten(), y.flatten(), z.flatten()))
        random_points = draw.get_random_points(n=5, scale=4)
        points_list = draw.get_bezier_curve(random_points, rad=0.2, edgy=0.05)
        cnt = len(points_list)
    elif mode == "random":
        center_list = [
            np.array([-30, 0]),
            np.array([-15, 0]),
            np.array([0, 0]),
            np.array([15, 0]),
            np.array([30, 0]),
            np.array([-30, 0]),
            np.array([-15, 0]),
            np.array([0, 0]),
            np.array([15, 0]),
            np.array([30, 0]),
        ]
        points_list = []
        circle_list = [draw.draw_circle(center, 6, 10) for center in center_list]
        for i in range(row_num):
            random_points = draw.get_random_points(n=5, scale=4)
            points = draw.get_bezier_curve(random_points, rad=0.2, edgy=0.05)
            points_list.append(points)
        cnt = min([len(points) for points in points_list])
        z_list = [3, 3, 3, 3, 3, 6, 6, 6, 6, 6]
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if mode == "circle":
            radius = 4
            for k in range(cnt):
                for i in range(row_num):
                    for j in range(column_num):
                        angle = 2 * math.pi / (column_num)
                        poses_msg.name[i * row_num + j] = (
                            vehicle_type + "_" + str(i * row_num + j)
                        )
                        poses_msg.pose[i * row_num + j].position.x = points_list[i][k][
                            0
                        ] + radius * math.cos(j * angle)
                        poses_msg.pose[i * row_num + j].position.y = points_list[i][k][
                            1
                        ] + radius * math.sin(j * angle)
                        poses_msg.pose[i * row_num + j].position.z = 3
                pub.publish(poses_msg)
                rate.sleep()
        elif mode == "sphere":
            for k in range(cnt):
                for i in range(row_num * column_num):
                    poses_msg.name[i] = vehicle_type + "_" + str(i)
                    poses_msg.pose[i].position.x = points_list[k][0] + center_list[i][0]
                    poses_msg.pose[i].position.y = points_list[k][1] + center_list[i][0]
                    poses_msg.pose[i].position.z = center_list[i][2]
                pub.publish(poses_msg)
                rate.sleep()
        elif mode == "random":
            for k in range(cnt):
                for i in range(row_num):
                    for j in range(column_num):
                        poses_msg.name[i * row_num + j] = (
                            vehicle_type + "_" + str(i * row_num + j)
                        )
                        poses_msg.pose[i * row_num + j].position.x = (
                            points_list[i][k][0] + circle_list[i][j][0]
                        )
                        poses_msg.pose[i * row_num + j].position.y = (
                            points_list[i][k][1] + circle_list[i][j][1]
                        )
                        poses_msg.pose[i * row_num + j].position.z = z_list[i]
                pub.publish(poses_msg)
                rate.sleep()


if __name__ == "__main__":
    rospy.init_node("pose_publisher")
    # TODO 增加请求应答节点，杀死损坏节点
    try:
        pose_publisher(mode="random")
    except rospy.ROSInterruptException:
        pass
