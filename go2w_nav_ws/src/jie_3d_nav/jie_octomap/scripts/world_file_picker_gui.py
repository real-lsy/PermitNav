#!/usr/bin/env python3

import os
import signal
import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WorldFilePickerNode(Node):
    def __init__(self):
        super().__init__("world_file_picker_gui")
        self.declare_parameter("world_file_cmd_topic", "/world_file_cmd")
        topic = self.get_parameter("world_file_cmd_topic").get_parameter_value().string_value
        self.pub = self.create_publisher(String, topic, 10)
        self.get_logger().info(f"World file picker publishing to {topic}")

    def publish_world(self, path: str):
        msg = String()
        msg.data = path
        self.pub.publish(msg)
        self.get_logger().info(f"Published world file: {path}")


class PickerWindow(QMainWindow):
    def __init__(self, node: WorldFilePickerNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Select World File")
        self.resize(820, 120)

        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        row = QHBoxLayout()
        row.addWidget(QLabel("World File:"))
        self.path_edit = QLineEdit()
        self.path_edit.setPlaceholderText("Select .world/.sdf and click Load")
        row.addWidget(self.path_edit, 1)
        browse_btn = QPushButton("Browse")
        browse_btn.clicked.connect(self._browse)
        row.addWidget(browse_btn)
        layout.addLayout(row)

        btn_row = QHBoxLayout()
        load_btn = QPushButton("Load To Current OctoMap")
        load_btn.clicked.connect(self._load_world)
        btn_row.addWidget(load_btn)
        layout.addLayout(btn_row)

    def _browse(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Select Gazebo World/SDF",
            "",
            "World Files (*.world *.sdf);;All Files (*)",
        )
        if filename:
            self.path_edit.setText(filename)

    def _load_world(self):
        path = self.path_edit.text().strip()
        if not path:
            QMessageBox.warning(self, "Missing File", "Please select a world file first.")
            return
        if not os.path.isfile(path):
            QMessageBox.warning(self, "Invalid File", f"File not found:\n{path}")
            return
        self.node.publish_world(path)


def main():
    rclpy.init()
    node = WorldFilePickerNode()

    app = QApplication(sys.argv)
    win = PickerWindow(node)
    win.show()

    timer = QTimer()

    def on_spin_once():
        if not rclpy.ok():
            timer.stop()
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            # ROS context may already be shutting down (e.g. Ctrl-C via launch).
            timer.stop()
            app.quit()

    def on_signal(_sig, _frame):
        timer.stop()
        app.quit()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    timer.timeout.connect(on_spin_once)
    timer.start(20)

    code = app.exec_()
    timer.stop()
    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(code)


if __name__ == "__main__":
    main()
