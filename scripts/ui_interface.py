import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QGridLayout, QSizePolicy
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtCore import QTimer, QThread, pyqtSlot, pyqtSignal, QObject
import pyqtgraph as pg

import rospy
from control.msg import Toughness
from globals import *


class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
        super(MplCanvas, self).__init__(self.fig)
        self.setParent(parent)
        FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)


class ROSWorker(QObject):
    updated = pyqtSignal(Toughness)

    def __init__(self):
        super().__init__()

    def start(self):
        rospy.Subscriber("ui/toughness", Toughness, self.callback, queue_size=1)

    def callback(self, msg):
        self.updated.emit(msg)


class RealTimePlot(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(100, 100, 1024, 1024)
        self.setWindowTitle("Real Time Data Visualization")

        # Layout setup
        layout = QGridLayout()
        self.setLayout(layout)

        # Dynamic matplotlib plot
        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        layout.addWidget(self.canvas, 0, 0)

        # Data initialization
        self.yts_raw = []
        self.yts = []
        self.max_length_of_msgs = []
        self.average_length_of_msgs = []
        self.clear_msgs = []
        self.time = []
        self.data2 = []
        self.data3 = []
        self.matrix = np.zeros((NUM, NUM), dtype=int)

        # Create plot widgets
        self.plot_num_of_msgs = pg.PlotWidget()
        self.plot_length_of_msgs = pg.PlotWidget()
        self.plot_clear_msgs = pg.PlotWidget()

        layout.addWidget(self.plot_num_of_msgs, 0, 1)
        layout.addWidget(self.plot_length_of_msgs, 1, 0)
        layout.addWidget(self.plot_clear_msgs, 1, 1)

        # Initialize matplotlib animation
        self.ani = animation.FuncAnimation(
            self.canvas.figure, self.update_plot, frames=100, interval=1000, blit=False
        )

    @pyqtSlot(Toughness)
    def update(self, msg):
        # Update data
        self.time.append(msg.time)
        self.data2.append(msg.yt_raw)
        self.data3.append(msg.yt)

        # Update plots
        self.plot_num_of_msgs.plot(self.time, self.yts, clear=True)
        self.plot_length_of_msgs.plot(self.time, self.data2, clear=True)
        self.plot_clear_msgs.plot(self.time, self.data3, clear=True)

    def update_plot(self, frame):
        self.canvas.ax.clear()

        # Your provided code for drawing
        lines = self.canvas.ax.lines  # 获取当前图形中的所有线条对象
        for line in lines:
            line.remove()  # 删除所有线条对象

        # Compute node positions on the circle
        angle = 2 * np.pi / NUM
        radius = 0.4  # 圆的半径
        node_positions = [
            (0.5 + radius * np.cos(i * angle), 0.5 + radius * np.sin(i * angle))
            for i in range(NUM)
        ]

        # Plot edges based on the adjacency matrix
        for i in range(NUM):
            for j in range(i + 1, NUM):
                if self.matrix[i][j] == 1:
                    self.canvas.ax.plot(
                        [node_positions[i][0], node_positions[j][0]],
                        [node_positions[i][1], node_positions[j][1]],
                        "black",
                    )

        for i in range(NUM):
            image = plt.imread(
                os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), "pic/plane.png"
                )
            )
            imagebox = OffsetImage(image, zoom=0.1)
            ab = AnnotationBbox(imagebox, node_positions[i], frameon=False)
            self.canvas.ax.add_artist(ab)

            # 添加节点名称的文本
            self.canvas.ax.text(
                node_positions[i][0],
                node_positions[i][1] + 0.05,
                f"Node {i}",
                ha="center",
                va="center",
                fontsize=12,
            )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = RealTimePlot()

    # Initialize ROS node
    rospy.init_node("ui_interface", anonymous=True)

    # Create ROS worker and move it to a separate thread
    ros_worker = ROSWorker()
    ros_thread = QThread()
    ros_worker.moveToThread(ros_thread)

    # Connect the signal from ROS worker to the slot in RealTimePlot
    ros_worker.updated.connect(ex.update)

    # Start ROS worker in a separate thread
    ros_thread.started.connect(ros_worker.start)
    ros_thread.start()

    ex.show()
    sys.exit(app.exec_())
