import signal
import sys
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QTimer, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QPlainTextEdit,
    QSlider,
    QSpinBox,
    QDoubleSpinBox,
    QVBoxLayout,
    QWidget,
)

from am_operator_gui.process_manager import ProcessRegistry
from am_operator_gui.ros_bridge import RosBridge


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_COMPONENTS_DIR = REPO_ROOT / 'components'
DEFAULT_TRAJECTORY_DIR = DEFAULT_COMPONENTS_DIR / 'robotnik_paired_demo'
LAUNCH_ALL_NAME = 'launch_all'
PUBLISH_PATH_NAME = 'publish_path'
MOVE_BASE_NAME = 'move_base_to_start'
RVIZ_NAME = 'rviz'


class OperatorWindow(QMainWindow):
    ros_status_changed = pyqtSignal(bool, bool)
    process_output = pyqtSignal(str, str)

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle('AM Operator GUI')
        self.resize(980, 720)
        self._has_path = False
        self._has_robot_pose = False

        self.processes = ProcessRegistry(output_callback=self._on_process_output)
        self.ros_bridge = RosBridge(status_callback=self._on_ros_status)

        self.ros_status_changed.connect(self._set_ros_status)
        self.process_output.connect(self._append_process_output)

        self._build_ui()
        self._connect_signals()
        self.ros_bridge.start()

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self._refresh_process_states)
        self.status_timer.start(500)

        self._publish_overrides()
        self._refresh_process_states()

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QVBoxLayout(root)
        layout.setSpacing(10)

        launch_group = QGroupBox('Launch')
        launch_layout = QGridLayout(launch_group)
        self.simulation_checkbox = QCheckBox('Simulation')
        self.direction_mode = QComboBox()
        self.direction_mode.addItems(['goal_direction', 'speed_orthogonal'])
        self.direction_mode.setCurrentText('goal_direction')
        self.index_spin = QSpinBox()
        self.index_spin.setRange(0, 100000)
        self.index_spin.setValue(0)

        self.path_folder = QLineEdit(str(DEFAULT_TRAJECTORY_DIR))
        self.path_folder.setReadOnly(True)
        self.browse_button = QPushButton('Browse')

        self.launch_button = QPushButton('Launch All')
        self.publish_path_button = QPushButton('Publish Path')
        self.move_base_button = QPushButton('Move Base To Start')
        self.start_following_button = QPushButton('Start Following')
        self.rviz_button = QPushButton('Open RViz')

        launch_layout.addWidget(self.simulation_checkbox, 0, 0)
        launch_layout.addWidget(QLabel('Direction'), 0, 1)
        launch_layout.addWidget(self.direction_mode, 0, 2)
        launch_layout.addWidget(QLabel('Current index'), 0, 3)
        launch_layout.addWidget(self.index_spin, 0, 4)
        launch_layout.addWidget(QLabel('Path folder'), 1, 0)
        launch_layout.addWidget(self.path_folder, 1, 1, 1, 3)
        launch_layout.addWidget(self.browse_button, 1, 4)
        launch_layout.addWidget(self.launch_button, 2, 0)
        launch_layout.addWidget(self.publish_path_button, 2, 1)
        launch_layout.addWidget(self.move_base_button, 2, 2)
        launch_layout.addWidget(self.start_following_button, 2, 3)
        launch_layout.addWidget(self.rviz_button, 2, 4)

        override_group = QGroupBox('Overrides')
        override_layout = QGridLayout(override_group)
        self.velocity_slider = QSlider(Qt.Horizontal)
        self.velocity_slider.setRange(0, 200)
        self.velocity_slider.setValue(100)
        self.velocity_value = QLabel('100%')

        self.nozzle_reference = QDoubleSpinBox()
        self.nozzle_reference.setRange(-10000.0, 10000.0)
        self.nozzle_reference.setDecimals(1)
        self.nozzle_reference.setSuffix(' mm')
        self.nozzle_reference.setValue(0.0)
        self.nozzle_offset = QSlider(Qt.Horizontal)
        self.nozzle_offset.setRange(-100, 100)
        self.nozzle_offset.setValue(0)
        self.nozzle_offset_value = QLabel('+0 mm')
        self.nozzle_effective_value = QLabel('0.0 mm effective')

        override_layout.addWidget(QLabel('Velocity override'), 0, 0)
        override_layout.addWidget(self.velocity_slider, 0, 1)
        override_layout.addWidget(self.velocity_value, 0, 2)
        override_layout.addWidget(QLabel('Nozzle reference'), 1, 0)
        override_layout.addWidget(self.nozzle_reference, 1, 1)
        override_layout.addWidget(self.nozzle_effective_value, 1, 2)
        override_layout.addWidget(QLabel('Nozzle offset'), 2, 0)
        override_layout.addWidget(self.nozzle_offset, 2, 1)
        override_layout.addWidget(self.nozzle_offset_value, 2, 2)

        status_group = QGroupBox('Status')
        status_layout = QHBoxLayout(status_group)
        self.path_status = QLabel('/base_path: waiting')
        self.pose_status = QLabel('/robot_pose: waiting')
        status_layout.addWidget(self.path_status)
        status_layout.addWidget(self.pose_status)
        status_layout.addStretch(1)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1000)

        layout.addWidget(launch_group)
        layout.addWidget(override_group)
        layout.addWidget(status_group)
        layout.addWidget(self.log, 1)
        self.setCentralWidget(root)

    def _connect_signals(self) -> None:
        self.browse_button.clicked.connect(self._choose_path_folder)
        self.launch_button.clicked.connect(self._toggle_launch_all)
        self.publish_path_button.clicked.connect(self._publish_path)
        self.move_base_button.clicked.connect(self._move_base_to_start)
        self.start_following_button.clicked.connect(self._start_following)
        self.rviz_button.clicked.connect(self._open_rviz)
        self.index_spin.valueChanged.connect(self._publish_path_index)
        self.velocity_slider.valueChanged.connect(self._publish_overrides)
        self.nozzle_reference.valueChanged.connect(self._publish_overrides)
        self.nozzle_offset.valueChanged.connect(self._publish_overrides)

    def _choose_path_folder(self) -> None:
        folder = QFileDialog.getExistingDirectory(
            self,
            'Select trajectory folder',
            str(DEFAULT_COMPONENTS_DIR),
        )
        if folder:
            self.path_folder.setText(folder)

    def _toggle_launch_all(self) -> None:
        process = self.processes.get(LAUNCH_ALL_NAME)
        if process is not None and process.is_running():
            self.processes.stop(LAUNCH_ALL_NAME)
            self._append_process_output(LAUNCH_ALL_NAME, 'stopped by operator')
            self._refresh_process_states()
            return

        sim_value = 'true' if self.simulation_checkbox.isChecked() else 'false'
        command = [
            'ros2',
            'launch',
            'am_bringup',
            'rbvogui_paired_base_arm_demo.launch.py',
            f'launch_sim:={sim_value}',
            f'gui:={sim_value}',
            'use_exported_trajectories:=true',
            f'trajectory_directory:={self.path_folder.text()}',
            f'direction_control_mode:={self.direction_mode.currentText()}',
            f'initial_path_index:={self.index_spin.value()}',
        ]
        self._append_process_output(LAUNCH_ALL_NAME, ' '.join(command))
        self.processes.start(LAUNCH_ALL_NAME, command)
        self._refresh_process_states()

    def _publish_path(self) -> None:
        command = [
            'ros2',
            'launch',
            'parse_paths',
            'robotnik_base_arm_paths.launch.py',
            'load_exported_trajectories:=true',
            f'trajectory_directory:={self.path_folder.text()}',
            'publish_once:=false',
        ]
        self._append_process_output(PUBLISH_PATH_NAME, ' '.join(command))
        self.processes.start(PUBLISH_PATH_NAME, command)
        self._refresh_process_states()

    def _move_base_to_start(self) -> None:
        self.processes.stop(MOVE_BASE_NAME)
        command = [
            'ros2',
            'launch',
            'move_to_path_idx',
            'move_to_path_idx.launch.py',
            'path_topic:=/base_path',
            'robot_pose_topic:=/robot_pose',
            'robot_pose_type:=pose_stamped',
            'cmd_vel_topic:=/robot/robotnik_base_control/cmd_vel_unstamped',
            f'path_index:={self.index_spin.value()}',
            'publish_start_condition:=false',
            'use_sim_time:=true',
        ]
        self._append_process_output(MOVE_BASE_NAME, ' '.join(command))
        self.processes.start(MOVE_BASE_NAME, command)
        self._refresh_process_states()

    def _start_following(self) -> None:
        self._start_condition_publish_count = 5
        self._publish_start_condition_once()
        self._style_button(self.start_following_button, 'green')

    def _publish_start_condition_once(self) -> None:
        self.ros_bridge.publish_start_condition(True)
        self._append_process_output('ros', 'published /start_condition true')
        self._start_condition_publish_count -= 1
        if self._start_condition_publish_count > 0:
            QTimer.singleShot(200, self._publish_start_condition_once)

    def _open_rviz(self) -> None:
        rviz_config = Path(get_package_share_directory('am_operator_gui')) / 'rviz' / 'robotnik_operator.rviz'
        command = ['rviz2', '-d', str(rviz_config)]
        self._append_process_output(RVIZ_NAME, ' '.join(command))
        self.processes.start(RVIZ_NAME, command)
        self._refresh_process_states()

    def _publish_path_index(self, value: int) -> None:
        self.ros_bridge.publish_path_index(value)
        self._append_process_output('ros', f'published /path_index {value}')

    def _publish_overrides(self) -> None:
        velocity_percent = self.velocity_slider.value()
        velocity_scale = velocity_percent / 100.0
        reference_mm = self.nozzle_reference.value()
        offset_mm = self.nozzle_offset.value()
        effective_mm = reference_mm + offset_mm
        self.velocity_value.setText(f'{velocity_percent}%')
        self.nozzle_offset_value.setText(f'{offset_mm:+d} mm')
        self.nozzle_effective_value.setText(f'{effective_mm:.1f} mm effective')
        self.ros_bridge.publish_velocity_override(velocity_scale)
        self.ros_bridge.publish_nozzle_height(effective_mm / 1000.0)

    def _on_process_output(self, name: str, line: str) -> None:
        self.process_output.emit(name, line)

    def _append_process_output(self, name: str, line: str) -> None:
        self.log.appendPlainText(f'[{name}] {line}')

    def _on_ros_status(self, has_path: bool, has_robot_pose: bool) -> None:
        self.ros_status_changed.emit(has_path, has_robot_pose)

    def _set_ros_status(self, has_path: bool, has_robot_pose: bool) -> None:
        self._has_path = has_path
        self._has_robot_pose = has_robot_pose
        self.path_status.setText('/base_path: ready' if has_path else '/base_path: waiting')
        self.pose_status.setText('/robot_pose: ready' if has_robot_pose else '/robot_pose: waiting')
        self._refresh_process_states()

    def _refresh_process_states(self) -> None:
        self._set_launch_button_state()
        self._set_publish_path_state()
        self._set_move_base_state()
        self._set_start_following_state()
        self._set_rviz_state()

    def _set_launch_button_state(self) -> None:
        process = self.processes.get(LAUNCH_ALL_NAME)
        running = process is not None and process.is_running()
        self.launch_button.setText('Stop All' if running else 'Launch All')
        self._style_button(self.launch_button, 'green' if running else 'grey')

    def _set_publish_path_state(self) -> None:
        process = self.processes.get(PUBLISH_PATH_NAME)
        if process is None:
            color = 'grey'
        elif process.is_running():
            color = 'green'
        elif process.return_code == 0:
            color = 'green'
        else:
            color = 'red'
        self._style_button(self.publish_path_button, color)

    def _set_move_base_state(self) -> None:
        process = self.processes.get(MOVE_BASE_NAME)
        if process is None:
            color = 'grey'
        elif process.is_running() and (not self._has_path or not self._has_robot_pose):
            color = 'orange'
        elif process.is_running():
            color = 'yellow'
        elif process.return_code == 0:
            color = 'green'
        else:
            color = 'red'
        self._style_button(self.move_base_button, color)

    def _set_start_following_state(self) -> None:
        if getattr(self, '_start_condition_publish_count', 0) > 0:
            return
        self._style_button(self.start_following_button, 'grey')

    def _set_rviz_state(self) -> None:
        process = self.processes.get(RVIZ_NAME)
        running = process is not None and process.is_running()
        self._style_button(self.rviz_button, 'green' if running else 'grey')

    @staticmethod
    def _style_button(button: QPushButton, color: str) -> None:
        colors = {
            'grey': '#d6d6d6',
            'green': '#53a653',
            'yellow': '#f0d35b',
            'orange': '#f2a541',
            'red': '#d95c5c',
        }
        background = colors.get(color, colors['grey'])
        button.setStyleSheet(
            f'QPushButton {{ background-color: {background}; color: #101010; padding: 6px; }}'
        )

    def closeEvent(self, event) -> None:
        self.processes.stop_all()
        self.ros_bridge.stop()
        event.accept()


def run(args: Optional[list[str]] = None) -> int:
    app = QApplication(args if args is not None else sys.argv)
    window = OperatorWindow()
    window.show()

    def _handle_sigint(_sig, _frame):
        window.close()

    signal.signal(signal.SIGINT, _handle_sigint)
    return app.exec_()
