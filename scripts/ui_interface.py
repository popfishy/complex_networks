import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.offsetbox import OffsetImage, AnnotationBbox
from PyQt5.QtWidgets import QApplication, QWidget, QGridLayout, QSizePolicy
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5.QtCore import QThread, pyqtSlot, pyqtSignal, QObject
import pyqtgraph as pg
import rospy
from control.msg import Toughness
from control.msg import Result
from globals import *


class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
        super(MplCanvas, self).__init__(self.fig)
        self.setParent(parent)
        FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)


class ROSWorker(QObject):
    updated_toughness = pyqtSignal(Toughness)
    updated_result = pyqtSignal(Result)

    def __init__(self):
        super().__init__()

    def start(self):
        rospy.Subscriber(
            "ui/toughness", Toughness, self.callback_toughness, queue_size=1
        )
        rospy.Subscriber("ui/result", Result, self.callback_result, queue_size=1)

    def callback_toughness(self, msg):
        self.updated_toughness.emit(msg)

    def callback_result(self, msg):
        self.updated_result.emit(msg)


class MplCanvas(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig, self.ax = plt.subplots(figsize=(width, height), dpi=dpi)
        super(MplCanvas, self).__init__(self.fig)
        self.setParent(parent)
        FigureCanvas.setSizePolicy(self, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)


class RealTimePlot(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(100, 100, 1536, 1024)
        self.setWindowTitle("ResearchGroup6 data visualization")

        # 启用抗锯齿选项
        pg.setConfigOptions(antialias=True)

        # Layout setup
        layout = QGridLayout()
        self.setLayout(layout)

        # Dynamic matplotlib plot
        self.canvas = MplCanvas(self, width=5, height=4, dpi=100)
        layout.addWidget(self.canvas, 0, 0)

        # Data initialization
        self.yts_raw = []
        self.yts = []
        # self.max_length_of_msgs = []
        # self.average_length_of_msgs = []
        self.clear_msgs = []
        self.time = []
        self.result = []
        self.final_result = 0.0
        self.matrix = np.zeros((NUM, NUM), dtype=int)

        # Create plot widgets
        self.plot_clear_msgs = pg.PlotWidget()
        self.plot_result_msgs = pg.PlotWidget()
        self.plot_num_of_msgs = pg.PlotWidget()
        self.plot_average_num_of_msgs = pg.PlotWidget()

        layout.addWidget(self.plot_clear_msgs, 0, 1)
        layout.addWidget(self.plot_result_msgs, 0, 2)
        layout.addWidget(self.plot_num_of_msgs, 1, 0)
        layout.addWidget(self.plot_average_num_of_msgs, 1, 1)

        # Set background color
        self.plot_num_of_msgs.setBackground("white")
        self.plot_average_num_of_msgs.setBackground("white")
        self.plot_clear_msgs.setBackground("white")
        self.plot_result_msgs.setBackground("white")

        # Set plot titles
        self.plot_clear_msgs.setTitle("Clear Messages", color="black")
        self.plot_result_msgs.setTitle("韧性评估值", color="black")
        self.plot_num_of_msgs.setTitle("Number of Messages", color="black")
        self.plot_average_num_of_msgs.setTitle(
            "Average Number of Messages", color="black"
        )

        # Set axis labels
        self.plot_num_of_msgs.setLabel("bottom", "时间:t/s", color="black")
        self.plot_num_of_msgs.setLabel("left", "消息数量", color="black")
        self.plot_average_num_of_msgs.setLabel("bottom", "时间:t/s", color="black")
        self.plot_average_num_of_msgs.setLabel("left", "滤波后消息数量", color="black")
        self.plot_clear_msgs.setLabel("bottom", "时间:t/s", color="black")
        self.plot_clear_msgs.setLabel("left", "无用消息", color="black")
        self.plot_result_msgs.setLabel("bottom", "毁伤次数", color="black")
        self.plot_result_msgs.setLabel("left", "单次韧性评估值", color="black")

        # Set font size
        # font = pg.QtGui.QFont()
        # font.setPixelSize(10)
        # self.plot_num_of_msgs.getAxis("bottom").setTickFont(font)
        # self.plot_num_of_msgs.getAxis("left").setTickFont(font)
        # self.plot_average_num_of_msgs.getAxis("bottom").setTickFont(font)
        # self.plot_average_num_of_msgs.getAxis("left").setTickFont(font)
        # self.plot_clear_msgs.getAxis("bottom").setTickFont(font)
        # self.plot_clear_msgs.getAxis("left").setTickFont(font)

        # Add region selection to self.plot_average_num_of_msgs
        # self.region = pg.LinearRegionItem()
        # self.plot_average_num_of_msgs.addItem(self.region)
        # self.region.sigRegionChanged.connect(self.update_region)

        # Initialize matplotlib animation
        self.ani = animation.FuncAnimation(
            self.canvas.figure, self.update_plot, frames=100, interval=1000, blit=False
        )

    @pyqtSlot(Toughness)
    def update_toughness(self, msg):
        # Update data
        self.matrix = np.array(msg.matrix).reshape(msg.rows, msg.cols)
        self.time.append(msg.time)
        self.clear_msgs.append(msg.clear_msgs)
        self.yts_raw.append(msg.yt_raw)
        self.yts.append(msg.yt)
        # self.result.append(msg.result)

        # Update plots
        self.plot_num_of_msgs.plot(
            self.time, self.yts_raw, pen=pg.mkPen(color="k"), clear=True
        )
        self.plot_average_num_of_msgs.plot(
            self.time, self.yts, pen=pg.mkPen(color="k"), clear=True
        )
        self.plot_clear_msgs.plot(
            self.time, self.clear_msgs, pen=pg.mkPen(color="k"), clear=True
        )

    @pyqtSlot(Result)
    def update_result(self, msg):
        self.result = msg.R
        if msg.R_total > 0:
            self.final_result = msg.R_total
        self.update_result_plot()

    def update_result_plot(self):
        self.plot_result_msgs.clear()

        x = np.arange(len(self.result))
        y = self.result
        if len(y) > 0:
            bg1 = pg.BarGraphItem(x=x, height=y, width=0.6, brush=(100, 149, 237))
            self.plot_result_msgs.addItem(bg1)

            for i in range(len(x)):
                text = pg.TextItem(
                    text=str("{:.2f}".format(y[i])), color="w", anchor=(0.5, -0.5)
                )
                text.setPos(x[i], y[i])
                self.plot_result_msgs.addItem(text)

            final_result_text = pg.TextItem(
                text=f"Avg: {self.final_result:.2f}", color="k", anchor=(1, 1)
            )
            font = pg.QtGui.QFont()
            font.setPixelSize(36)
            final_result_text.setFont(font)

            final_result_text.setPos(
                len(x) - 1, max(y) * 1.1
            )  # Adjust position as needed
            self.plot_result_msgs.addItem(final_result_text)

    def update_region(self):
        minX, maxX = self.region.getRegion()
        self.plot_average_num_of_msgs.setXRange(minX, maxX, padding=0)

        # Update the zoomed plot with data within the selected region
        minX_index = int(minX)
        maxX_index = int(maxX)

        if maxX_index > len(self.time):
            maxX_index = len(self.time)

        if minX_index < len(self.time):
            region_time = self.time[minX_index:maxX_index]
            region_avg_msgs = self.yts[minX_index:maxX_index]

            self.plot_average_num_of_msgs.clear()
            self.plot_average_num_of_msgs.plot(region_time, region_avg_msgs, pen="b")

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
                node_positions[i][0] + 0.03 * np.cos(angle * i),
                node_positions[i][1] + 0.03 * np.sin(angle * i),
                f"uav {i}",
                ha="center",
                va="center",
                fontsize=10,
            )

        self.canvas.ax.set_axis_off()
        self.canvas.draw()


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
    ros_worker.updated_toughness.connect(ex.update_toughness)
    ros_worker.updated_result.connect(ex.update_result)

    # Start ROS worker in a separate thread
    ros_thread.started.connect(ros_worker.start)
    ros_thread.start()

    ex.show()
    sys.exit(app.exec_())
