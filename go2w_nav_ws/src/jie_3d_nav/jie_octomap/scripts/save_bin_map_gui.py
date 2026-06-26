#!/usr/bin/env python3

import os
import signal
import sys

from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class SaveBinMapWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Save Bin Map")
        self.resize(760, 420)

        self._workdir = "/home/robot/ros2_ws/src/odin_ros_driver"
        self._process = QProcess(self)
        self._process.setProgram("/bin/bash")
        self._process.setArguments(["-lc", "./set_param.sh save_map 1"])
        self._process.setWorkingDirectory(self._workdir)
        self._process.readyReadStandardOutput.connect(self._on_stdout)
        self._process.readyReadStandardError.connect(self._on_stderr)
        self._process.finished.connect(self._on_finished)

        central = QWidget(self)
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.save_btn = QPushButton("保存bin地图")
        self.save_btn.clicked.connect(self._run_save_map)
        layout.addWidget(self.save_btn)

        self.output_edit = QTextEdit()
        self.output_edit.setReadOnly(True)
        layout.addWidget(self.output_edit, 1)

        self._append_line("Ready.")

    def _append_line(self, text: str):
        self.output_edit.append(text.rstrip("\n"))

    def _run_save_map(self):
        if self._process.state() != QProcess.NotRunning:
            self._append_line("Command is already running.")
            return

        self.output_edit.clear()
        self._append_line(f"$ cd {self._workdir}")
        if not os.path.isdir(self._workdir):
            self._append_line(f"Directory not found: {self._workdir}")
            return

        command_path = os.path.join(self._workdir, "set_param.sh")
        self._append_line("$ ./set_param.sh save_map 1")
        if not os.path.isfile(command_path):
            self._append_line(f"File not found: {command_path}")
            return

        if not os.access(command_path, os.X_OK):
            self._append_line(f"File is not executable: {command_path}")
            return

        self.save_btn.setEnabled(False)
        self._process.start()

    def _on_stdout(self):
        data = bytes(self._process.readAllStandardOutput()).decode("utf-8", errors="replace")
        if data:
            self._append_line(data)

    def _on_stderr(self):
        data = bytes(self._process.readAllStandardError()).decode("utf-8", errors="replace")
        if data:
            self._append_line(data)

    def _on_finished(self, exit_code: int, exit_status):
        self.save_btn.setEnabled(True)
        self._append_line(f"[exit_code={exit_code} status={int(exit_status)}]")


def main():
    app = QApplication(sys.argv)
    win = SaveBinMapWindow()
    win.show()

    def on_signal(_sig, _frame):
        app.quit()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
