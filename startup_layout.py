#!/usr/bin/env python3
"""Bảng điều khiển robot: bản đồ, waypoint và Nav2 trên ROS 2 Jazzy."""
import os
import json
import math
import time
import uuid
import signal
import sys
import html
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QPushButton, QLabel, QTextEdit, QMessageBox, QStackedWidget)
from PyQt6.QtCore import QEvent, QTimer, Qt, QSize
from PyQt6.QtGui import QFont
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
from config import SETTINGS, UI_DIRECTORY, shell_source_workspace, robot_command
from map_utils import get_current_map_path, get_current_map_name, load_map_yaml, update_map_files
from waypoint_store import load_waypoint_file, resolve_waypoint, save_waypoint_file, deletion_reason, route_references
from waypoint_dialogs import DestinationDialog, WaypointPickerDialog, NewWaypointDialog, PathManagerDialog, NewPathDialog, DIALOG_STYLE
from waypoint_map_widget import MapWidget
from startup_style import STARTUP_STYLESHEET, startup_text, repolish
from startup_icons import startup_icon
from ui_utils import setup_clock_timer
from new_map_layout import NewMapUI
from load_map_dialog import LoadMapDialog
from mapping_process import MappingProcess
from robot_eyes import RobotEyes

class RobotUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self._latest_pose = self._latest_amcl_msg = None
        self._nav_goal_handle = None
        self._nav_pending = self._nav_cancel_requested = False
        self._navigation_generation = 0
        self._waypoint_queue = []
        self._waypoints_file = str(UI_DIRECTORY / 'waypoints.json')
        self._waypoints = self._load_waypoints()
        self._stm32_last_msg_time = self._front_lidar_last_msg_time = self._rear_lidar_last_msg_time = None
        self.prev_stm32_status = self.prev_lidar_status = self.prev_lidar_rear_status = None
        self._nav_process = self._rviz_process = None
        self._stop_requested = False
        self._cancel_all_future = None
        self._arbiter = None
        self._arbiter_stamp = 0.0
        self._ros_node = Node('robot_ui_node')
        self._nav_client = ActionClient(self._ros_node, NavigateToPose, SETTINGS['navigate_action'])
        self._cancel_all = self._ros_node.create_client(CancelGoal, SETTINGS['navigate_action'] + '/_action/cancel_goal')
        self._stop_pub = self._ros_node.create_publisher(Bool, SETTINGS['emergency_stop_topic'], 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._ros_node.create_subscription(PoseWithCovarianceStamped, SETTINGS['pose_topic'], self._amcl_callback, qos)
        self._ros_node.create_subscription(OccupancyGrid, SETTINGS['map_topic'], self._map_callback, qos)
        self._ros_node.create_subscription(Odometry, SETTINGS['odom_topic'], self._stm32_odom_callback, qos_profile_sensor_data)
        for side, key in (('front','front_scan_topic'), ('rear','rear_scan_topic')):
            self._ros_node.create_subscription(LaserScan, SETTINGS[key], lambda msg, side=side: self._lidar_callback(side), qos_profile_sensor_data)
        self._ros_node.create_subscription(String, SETTINGS['arbiter_state_topic'], self._arbiter_callback, 1)
        self.init_ui()
        self._idle_timer = QTimer(self)
        self._idle_timer.setSingleShot(True)
        self._idle_timer.setInterval(10_000)
        self._idle_timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._idle_timer.timeout.connect(self._enter_idle)
        QApplication.instance().installEventFilter(self)
        self._idle_timer.start()
        self._ros_spin_timer = QTimer(self)
        self._ros_spin_timer.timeout.connect(self._spin)
        self._ros_spin_timer.start(30)
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(500)
        self.clock_timer = setup_clock_timer(self.clock_label)
        self._refresh_map_window()

    def init_ui(self):
        self.setWindowTitle('Bảng điều khiển robot')
        self.resize(1920, 1080)
        self.setStyleSheet(DIALOG_STYLE + STARTUP_STYLESHEET)
        central = QWidget()
        central.setObjectName('startup-root')
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0,0,0,0)
        sidebar = QWidget()
        sidebar.setObjectName('left-panel')
        sidebar.setFixedWidth(224)
        menu = QVBoxLayout(sidebar)
        menu.setContentsMargins(16,20,16,16)
        brand = QLabel('ROBOT')
        brand.setObjectName('wordmark')
        menu.addWidget(brand)
        title = QLabel('Tổng quan')
        title.setObjectName('overview-label')
        title.setMinimumHeight(44)
        menu.addWidget(title)
        for text, icon, callback in [('Địa điểm và lộ trình','waypoints',self.open_destination_dialog),
                ('Tải bản đồ','load_map',self.load_map), ('Tạo bản đồ','new_map',self.start_new_map),
                ('Bật Nav2 và RViz','nav2',self.start_nav2),
                ('Tắt Nav2 và RViz','nav2',self.stop_nav2)]:
            button = QPushButton(text)
            button.setObjectName('mode-btn')
            button.setMinimumHeight(52)
            button.setIcon(startup_icon(icon))
            button.setIconSize(QSize(22,22))
            button.clicked.connect(callback)
            menu.addWidget(button)
        menu.addStretch()
        self.stop_button = QPushButton('■  DỪNG')
        self.stop_button.setObjectName('interrupt-btn')
        self.stop_button.setMinimumHeight(64)
        self.stop_button.clicked.connect(self.stop_robot)
        menu.addWidget(self.stop_button)
        self.resume_button = QPushButton('Cho phép chạy lại')
        self.resume_button.setMinimumHeight(48)
        self.resume_button.clicked.connect(self.resume_robot)
        menu.addWidget(self.resume_button)
        hint = QLabel('Đặt vị trí ban đầu của robot trong RViz.')
        hint.setWordWrap(True)
        menu.addWidget(hint)
        layout.addWidget(sidebar)
        right = QWidget()
        right.setObjectName('content-panel')
        body = QVBoxLayout(right)
        body.setContentsMargins(24,20,24,20)
        header = QHBoxLayout()
        title = QLabel('Bảng điều khiển robot')
        title.setObjectName('header-title')
        header.addWidget(title,1)
        self.clock_label = QLabel()
        self.clock_label.setObjectName('clock')
        header.addWidget(self.clock_label)
        body.addLayout(header)
        cards = QHBoxLayout()
        self.stm32_card = self._make_status_card('STM32')
        self.front_lidar_card = self._make_status_card('front')
        self.rear_lidar_card = self._make_status_card('rear')
        for card in (self.stm32_card,self.front_lidar_card,self.rear_lidar_card):
            cards.addWidget(card['widget'],1)
        body.addLayout(cards)
        self.navigation_label = QLabel('Điều hướng: chưa khởi động')
        body.addWidget(self.navigation_label)
        self._map_view = MapWidget('', {'resolution':1.0, 'origin':[0,0,0]})
        self._display = QStackedWidget()
        self._eyes = RobotEyes()
        self._display.addWidget(self._map_view)
        self._display.addWidget(self._eyes)
        body.addWidget(self._display, 1)
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumHeight(120)
        self.log_box.document().setMaximumBlockCount(250)
        body.addWidget(self.log_box)
        layout.addWidget(right,1)

    def _idle_blocked(self):
        mapping = getattr(self, '_new_map_dialog', None)
        return (not self.isVisible() or self._stop_requested or self._navigation_busy()
                or bool(self._arbiter and (self._arbiter.get('navigation_busy')
                                           or self._arbiter.get('emergency_stop')))
                or QApplication.activeModalWidget() is not None
                or bool(mapping and (mapping.isVisible() or mapping.mapping_process is not None)))

    def _enter_idle(self):
        if self._idle_blocked():
            self._idle_timer.start()
            return
        self._display.setCurrentWidget(self._eyes)

    def _return_to_dashboard(self):
        self._display.setCurrentWidget(self._map_view)
        self._idle_timer.start()

    def eventFilter(self, watched, event):
        wake_events = (QEvent.Type.MouseButtonPress, QEvent.Type.MouseButtonDblClick,
                       QEvent.Type.Wheel, QEvent.Type.KeyPress, QEvent.Type.TouchBegin)
        if event.type() in wake_events:
            if self._display.currentWidget() is self._eyes:
                self._return_to_dashboard()
                # A click on the visible Stop button must still reach the button.
                if watched is not self.stop_button:
                    return True
            else:
                self._idle_timer.start()
        return super().eventFilter(watched, event)

    def _spin(self):
        rclpy.spin_once(self._ros_node, timeout_sec=0)
        if self._stop_requested:
            self._stop_pub.publish(Bool(data=True))

    def log(self, message):
        print(message)
        if hasattr(self, 'log_box'):
            self.log_box.append(html.escape(str(message)))

    def show_stm32_diagnostics(self):
        self.log('Trạng thái STM32 được xác định từ dữ liệu odometry: ' + SETTINGS['odom_topic'])

    def _amcl_callback(self, msg):
        if msg.header.frame_id.lstrip('/') != SETTINGS['map_frame'].lstrip('/'):
            return
        self._latest_amcl_msg = msg
        self._latest_pose = msg.pose.pose
        self._map_view.set_robot_pose(msg)

    def _map_callback(self, msg):
        if msg.header.frame_id.lstrip('/') == SETTINGS['map_frame'].lstrip('/'):
            self._map_view.set_occupancy_grid(msg)

    def _arbiter_callback(self, msg):
        try:
            state = json.loads(msg.data)
            if not isinstance(state, dict) or type(state.get('emergency_stop')) is not bool:
                return
            self._arbiter = state
            self._arbiter_stamp = time.monotonic()
        except (ValueError, TypeError):
            pass

    def stop_robot(self):
        self._return_to_dashboard()
        self._stop_requested = True
        self._stop_pub.publish(Bool(data=True))
        self.cancel_navigation()
        if self._cancel_all.service_is_ready():
            if self._cancel_all_future is None or self._cancel_all_future.done():
                self._cancel_all_future = self._cancel_all.call_async(CancelGoal.Request())
                self._cancel_all_future.add_done_callback(self._cancel_all_done)
        self.navigation_label.setText('Đã yêu cầu dừng; chờ xác nhận từ Nav2 và bộ phân xử vận tốc.')

    def resume_robot(self):
        if self._navigation_busy() or (self._cancel_all_future is not None and not self._cancel_all_future.done()):
            self.log('Chờ Nav2 xác nhận hủy trước khi cho phép chạy lại.')
            return
        if not self._arbiter or time.monotonic()-self._arbiter_stamp > 1.0:
            self.log('Không nhận được trạng thái bộ phân xử vận tốc.')
            return
        if self._arbiter.get('navigation_busy', True):
            self.log('Nav2 vẫn còn mục tiêu đang xử lý; chưa thể mở khóa dừng.')
            return
        self._stop_requested = False
        self._stop_pub.publish(Bool(data=False))
        self.log('Đã yêu cầu mở khóa dừng. Hành trình cũ không tự chạy tiếp.')

    def _cancel_all_done(self, future):
        try:
            result = future.result()
            if result.return_code not in (0, 2):
                self.log('Nav2 chưa xác nhận yêu cầu hủy; nhấn Dừng để thử lại.')
        except Exception as exc:
            self.log(f'Lỗi yêu cầu hủy các mục tiêu Nav2: {exc}')

    def stop_nav2(self):
        self.stop_robot()
        if self._navigation_busy():
            self.log('Đang hủy mục tiêu. Nhấn tắt Nav2 lại sau khi đã hủy.')
            return
        try:
            for attr in ('_nav_process', '_rviz_process'):
                proc = getattr(self, attr)
                if proc is not None:
                    proc.stop()
                    setattr(self, attr, None)
            self._latest_pose = self._latest_amcl_msg = None
            self._map_view.set_robot_pose(None)
            self.log('Đã tắt Nav2/RViz do giao diện khởi chạy. Robot vẫn khóa dừng.')
        except Exception as exc:
            self.log(f'Không thể tắt Nav2/RViz: {exc}')

    def start_nav2(self):
        if self._nav_process is not None and self._nav_process.poll() is None:
            self.log('Nav2 do giao diện khởi chạy vẫn đang hoạt động.')
            return
        if not get_current_map_path():
            self.log('Hãy tải bản đồ trước khi bật Nav2.')
            return
        if self._nav_client.server_is_ready():
            self.log('Nav2 đã chạy bên ngoài giao diện; không khởi chạy trùng.')
            return
        mapping = getattr(self, '_new_map_dialog', None)
        if mapping is not None and mapping.mapping_process is not None:
            self.log('Hãy kết thúc tạo bản đồ trước khi bật Nav2.')
            return
        try:
            self._nav_process = MappingProcess(shell_source_workspace(robot_command('nav2_command', map=get_current_map_path())))
            if not SETTINGS['nav2_opens_rviz']:
                self._rviz_process = MappingProcess(shell_source_workspace(SETTINGS['rviz_command']))
            self.log('Đã gửi lệnh bật Nav2 và RViz. Đặt vị trí ban đầu trong RViz.')
        except Exception as exc:
            self.log(f'Không thể khởi chạy Nav2/RViz: {exc}')

    def _refresh_map_window(self):
        try:
            path = get_current_map_path()
            if path:
                data = load_map_yaml(path)
                self._map_view.load_map(os.path.join(os.path.dirname(path), data['image']), data)
            self._waypoints = self._load_waypoints()
            self._map_view.set_waypoints({k:v for k,v in self._waypoints.items() if v['map_name']==get_current_map_name()})
            self._map_view.set_robot_pose(self._latest_amcl_msg)
            return True
        except Exception as exc:
            self.log(f'Không thể hiển thị bản đồ: {exc}')
            return False

    def closeEvent(self, event):
        self.stop_robot()
        deadline = time.monotonic()+3.0
        while self._navigation_busy() and time.monotonic()<deadline:
            rclpy.spin_once(self._ros_node, timeout_sec=0.05)
        if self._navigation_busy():
            self.log('Chưa xác nhận Nav2 đã hủy. Chờ rồi đóng lại giao diện.')
            event.ignore()
            return
        dialog = getattr(self, '_new_map_dialog', None)
        if dialog is not None and not dialog.close():
            event.ignore()
            return
        try:
            for proc in (self._nav_process,self._rviz_process):
                if proc is not None:
                    proc.stop()
        except Exception as exc:
            self.log(f'Chưa dừng được tiến trình: {exc}')
            event.ignore()
            return
        self._ros_spin_timer.stop()
        self._ros_node.destroy_node()
        event.accept()

    def _stm32_odom_callback(self, msg):
        self._stm32_last_msg_time = time.monotonic()

    def _lidar_callback(self, which):
        if which == 'front':
            self._front_lidar_last_msg_time = time.monotonic()
        else:
            self._rear_lidar_last_msg_time = time.monotonic()

    def _make_status_card(self, device_name):
        card = QWidget()
        card.setObjectName("status-card")
        card.setProperty("state", "checking")
        card.setMinimumHeight(104)
        layout = QHBoxLayout(card)
        layout.setContentsMargins(8, 12, 8, 12)
        layout.setSpacing(6)
        dot = QLabel()
        dot.setObjectName("device-icon")
        dot.setFixedSize(28, 28)
        dot.setAlignment(Qt.AlignmentFlag.AlignCenter)
        dot.setPixmap(startup_icon(device_name, size=24).pixmap(QSize(24, 24)))
        info = QVBoxLayout()
        name = QLabel(device_name)
        name.setObjectName("device-name")
        name.setWordWrap(True)
        state = QLabel()
        state.setObjectName("device-state")
        state.setWordWrap(True)
        info.addWidget(name)
        info.addWidget(state)
        layout.addWidget(dot)
        layout.addLayout(info, 1)
        detail = None
        if device_name == "STM32":
            detail = QPushButton("i")
            detail.setObjectName("diagnostics-btn")
            detail.setFixedSize(48, 48)
            detail.clicked.connect(self.show_stm32_diagnostics)
            layout.addWidget(detail)
        result = dict(widget=card, dot=dot, state=state, name=name,
                      device=device_name, available=None, detail_btn=detail)
        self._set_card_status(result, None)
        return result

    def _set_card_status(self, card, available):
        card["available"] = available
        state = "checking" if available is None else "online" if available else "offline"
        t = startup_text()
        card["state"].setText(t[state])
        card["name"].setText(t.get(card["device"], card["device"]))
        card["widget"].setAccessibleName(card["name"].text() + ": " + t[state])
        if card["widget"].property("state") != state:
            card["widget"].setProperty("state", state)
            repolish(card["widget"])
        if card["detail_btn"] is not None:
            card["detail_btn"].setToolTip(t["diagnostics"])
            card["detail_btn"].setAccessibleName(t["diagnostics"])

    def update_status(self):
        if self._idle_blocked():
            self._return_to_dashboard()
        now = time.monotonic()
        ready = self._nav_client.server_is_ready()
        fresh = self._arbiter is not None and now - self._arbiter_stamp < 1.0
        if self._stop_requested:
            confirmed = fresh and self._arbiter.get('emergency_stop') and not self._navigation_busy() and not self._arbiter.get('navigation_busy', True)
            message = 'Đã xác nhận khóa dừng' if confirmed else 'Đang yêu cầu dừng; chưa đủ xác nhận'
        elif fresh and self._arbiter.get('emergency_stop'):
            message = 'Đang khóa dừng tại bộ phân xử vận tốc'
        else:
            message = 'Nav2 sẵn sàng' if ready else 'Nav2 chưa sẵn sàng'
        self.navigation_label.setText('Điều hướng: ' + message)
        for attr in ('_nav_process', '_rviz_process'):
            proc = getattr(self, attr)
            if proc is not None and proc.poll() is not None:
                self.log('Tiến trình đã kết thúc: ' + proc.error_detail())
                proc.stop()
                setattr(self, attr, None)

        stm32_available = (
            self._stm32_last_msg_time is not None and
            now - self._stm32_last_msg_time < 3.0
        )
        self._set_card_status(self.stm32_card, stm32_available)
        if self.prev_stm32_status is not None and self.prev_stm32_status != stm32_available:
            self.log("Mất kết nối STM32" if not stm32_available else "Đã khôi phục kết nối STM32")
        self.prev_stm32_status = stm32_available

        front_available = (
            self._front_lidar_last_msg_time is not None and
            now - self._front_lidar_last_msg_time < 3.0
        )
        self._set_card_status(self.front_lidar_card, front_available)
        if self.prev_lidar_status is not None and self.prev_lidar_status != front_available:
            self.log("Mất kết nối LiDAR trước" if not front_available else "Đã khôi phục kết nối LiDAR trước")
        self.prev_lidar_status = front_available

        rear_available = (
            self._rear_lidar_last_msg_time is not None and
            now - self._rear_lidar_last_msg_time < 3.0
        )
        self._set_card_status(self.rear_lidar_card, rear_available)
        if self.prev_lidar_rear_status is not None and self.prev_lidar_rear_status != rear_available:
            self.log("Mất kết nối LiDAR sau" if not rear_available else "Đã khôi phục kết nối LiDAR sau")
        self.prev_lidar_rear_status = rear_available

    def _load_waypoints(self):
        try:
            data, _ = load_waypoint_file(self._waypoints_file)

            self.log(
                f"Đã tải {len(data)} địa điểm"
            )

            return data

        except Exception as e:
            self.log(
                f"Lỗi đọc waypoints.json: {e}"
            )
            return {}

    def start_new_map(self):
        if self._nav_client.server_is_ready():
            self.log('Hãy tắt Nav2 trước khi tạo bản đồ.')
            return
        if getattr(self, '_new_map_dialog', None) is None:
            self._new_map_dialog = NewMapUI(self)
        self._new_map_dialog.show()
        self._new_map_dialog.raise_()
        self._new_map_dialog.activateWindow()

    def load_map(self):
        if self._nav_client.server_is_ready() or self._navigation_busy():
            self.log('Hãy tắt Nav2 trước khi đổi bản đồ.')
            return
        dialog = LoadMapDialog(self)
        if dialog.exec():
            map_name = dialog.get_selected_map()
            if map_name:
                self.log(f"Đang tải bản đồ: {map_name}")
                self.cancel_navigation()
                if update_map_files(map_name, self.log):
                    self._latest_pose = None
                    self._latest_amcl_msg = None
                    self._waypoints = self._load_waypoints()
                    self._refresh_map_window()

    def open_destination_dialog(self):
        if getattr(self, '_destination_dialog', None) is None:
            self._destination_dialog = DestinationDialog(self)
        self._destination_dialog.show()
        self._destination_dialog.raise_()
        self._destination_dialog.activateWindow()

    def open_waypoint_picker(self):
        self._waypoints = self._load_waypoints()
        dialog = WaypointPickerDialog(self._waypoints, get_current_map_name(), self, self.delete_waypoint)
        if dialog.exec() and dialog.get_selected_key():
            self._run_waypoint_sequence([dialog.get_selected_key()])

    def open_path_manager(self):
        path = os.path.join(str(UI_DIRECTORY), 'multi_waypoints.json')
        while True:
            current_map = get_current_map_name()
            dialog = PathManagerDialog(path, current_map, self)
            dialog.run_path_requested.connect(self._run_waypoint_sequence)
            if dialog.exec() != 2:
                break
            self._waypoints = self._load_waypoints()
            NewPathDialog(self._waypoints, current_map, path, self).exec()

    def _run_waypoint_sequence(self, sequence):
        if self._stop_requested or not self._arbiter or time.monotonic()-self._arbiter_stamp > 1.0 or self._arbiter.get('emergency_stop') or not self._arbiter.get('healthy'):
            self.log('Bộ phân xử vận tốc chưa sẵn sàng hoặc đang khóa dừng.')
            return
        if self._navigation_busy():
            self.log('Đang chạy hoặc chờ hủy; hãy dừng và chờ kết quả trước khi chọn địa điểm mới.')
            return
        self._waypoints = self._load_waypoints()
        current_map = get_current_map_name()
        if (not isinstance(sequence, list) or not sequence
                or any(resolve_waypoint(self._waypoints, key, current_map) is None
                       for key in sequence)):
            QMessageBox.warning(self, 'Không thể điều hướng',
                                'Lộ trình trống hoặc có địa điểm không thuộc bản đồ hiện tại.')
            return
        # Keep keys as a list: waypoint names may contain commas.
        self.cancel_navigation()
        self._waypoint_queue = list(sequence)
        self._send_next_waypoint()

    def open_new_waypoint_dialog(self):
        if self._latest_pose is None:
            QMessageBox.warning(self, 'Chưa có vị trí',
                                'Robot chưa có vị trí AMCL. Hãy định vị robot trước khi tạo địa điểm.')
            return
        current_map = get_current_map_name()
        dialog = NewWaypointDialog(self)
        if not dialog.exec():
            return
        if self._latest_pose is None or current_map != get_current_map_name():
            QMessageBox.warning(self, 'Không thể lưu địa điểm',
                                'Bản đồ hoặc vị trí đã thay đổi. Hãy định vị và thử lại.')
            return
        name = dialog.get_name()
        try:
            data, snapshot = load_waypoint_file(self._waypoints_file)
            if (not name or name.startswith('__')
                    or any(name.casefold() == key.casefold() for key in data)):
                raise ValueError('Tên trống, trùng tên hoặc bắt đầu bằng __. Hãy chọn tên khác.')
            pose = self._latest_pose
            data[name] = {
                'id': str(uuid.uuid4()), 'display_name': name,
                'deletable': dialog.get_deletable(), 'map_name': current_map,
                'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z,
                'qx': pose.orientation.x, 'qy': pose.orientation.y,
                'qz': pose.orientation.z, 'qw': pose.orientation.w,
            }
            self._waypoints, _ = save_waypoint_file(self._waypoints_file, data, snapshot)
        except (OSError, ValueError, UnicodeError) as exc:
            QMessageBox.warning(self, 'Không thể lưu địa điểm', str(exc))
            return
        self._refresh_map_window()
        self.log(f'Đã lưu địa điểm mới: {name}')

    def _send_next_waypoint(self, generation=None):
        if generation is not None and generation != self._navigation_generation:
            return
        if self._stop_requested:
            self._waypoint_queue.clear()
            return
        generation = self._navigation_generation

        if not self._waypoint_queue:
            self.log(
                "[robot] Đã hoàn thành hành trình"
            )
            return

        target = self._waypoint_queue.pop(0)

        # ============================================================
        # Trường hợp quay lại vị trí lúc ra lệnh
        # ============================================================

        if target.startswith("__return_here__:"):

            try:
                values = target.split(":", 1)[1]
                x, y, qz, qw = map(
                    float,
                    values.split(";")
                )

                waypoint = {
                    "x": x,
                    "y": y,
                    "z": 0.0,
                    "qx": 0.0,
                    "qy": 0.0,
                    "qz": qz,
                    "qw": qw,
                }

                display_name = "vị trí ban đầu"

            except Exception as e:
                self.log(
                    f"Lỗi đọc __return_here__: {e}"
                )
                return

        # ============================================================
        # Waypoint bình thường
        # ============================================================

        else:

            self._waypoints = self._load_waypoints()
            key = resolve_waypoint(self._waypoints, target, get_current_map_name())
            waypoint = self._waypoints.get(key)

            if waypoint is None:
                self.log(
                    f"Không tìm thấy địa điểm: {target}"
                )
                return

            display_name = waypoint.get('display_name', target)

        # ============================================================
        # Kiểm tra Nav2
        # ============================================================

        if not self._nav_client.wait_for_server(
            timeout_sec=1.0
        ):
            self.log(
                "Nav2 chưa sẵn sàng: "
                "/navigate_to_pose không tồn tại"
            )
            return

        # ============================================================
        # Tạo goal
        # ============================================================

        goal = NavigateToPose.Goal()

        goal.pose.header.frame_id = SETTINGS['map_frame']

        goal.pose.header.stamp = (
            self._ros_node
            .get_clock()
            .now()
            .to_msg()
        )

        # Position
        goal.pose.pose.position.x = float(
            waypoint["x"]
        )

        goal.pose.pose.position.y = float(
            waypoint["y"]
        )

        goal.pose.pose.position.z = float(
            waypoint.get("z", 0.0)
        )

        # Orientation
        goal.pose.pose.orientation.x = float(
            waypoint.get("qx", 0.0)
        )

        goal.pose.pose.orientation.y = float(
            waypoint.get("qy", 0.0)
        )

        goal.pose.pose.orientation.z = float(
            waypoint["qz"]
        )

        goal.pose.pose.orientation.w = float(
            waypoint["qw"]
        )

        self.log(
            f"[robot] Đang đi tới {display_name} | "
            f"x={waypoint['x']:.2f}, "
            f"y={waypoint['y']:.2f}"
        )

        self._nav_pending = True
        self._nav_cancel_requested = False
        try:
            future = self._nav_client.send_goal_async(
                goal,
                feedback_callback=self._nav_feedback_callback
            )
        except Exception as exc:
            self._nav_pending = False
            self._waypoint_queue.clear()
            self.log(f'Lỗi gửi địa điểm: {exc}')
            return

        future.add_done_callback(
            lambda done: self._nav_goal_response_callback(done, generation)
        )

    def _nav_goal_response_callback(self, future, generation=None):
        self._nav_pending = False
        if generation is None:
            generation = self._navigation_generation

        try:
            goal_handle = future.result()
            if generation != self._navigation_generation:
                if goal_handle.accepted:
                    self._nav_goal_handle = goal_handle
                    goal_handle.get_result_async().add_done_callback(
                        lambda done: self._nav_result_callback(done, generation))
                    goal_handle.cancel_goal_async()
                return

        except Exception as e:
            self.log(
                f"Lỗi gửi mục tiêu Nav2: {e}"
            )
            self._waypoint_queue.clear()
            return

        if not goal_handle.accepted:

            self.log(
                "Nav2 từ chối địa điểm"
            )

            self._waypoint_queue.clear()
            return

        self._nav_goal_handle = goal_handle

        self.log(
            "Nav2 đã nhận địa điểm"
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            lambda done: self._nav_result_callback(done, generation)
        )

    def _nav_result_callback(self, future, generation=None):
        if generation is None:
            generation = self._navigation_generation
        if generation != self._navigation_generation:
            try:
                future.result()
                self._nav_goal_handle = None
                self.log('[robot] Mục tiêu đã kết thúc sau yêu cầu hủy.')
            except Exception as exc:
                self.log(f'Chưa xác nhận waypoint dừng: {exc}')
            return

        try:
            wrapped_result = future.result()
            status = wrapped_result.status

        except Exception as e:
            self.log(
                f"Lỗi nhận kết quả Nav2: {e}"
            )

            self._waypoint_queue.clear()
            return

        self._nav_goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:

            self.log(
                "[robot] Đã tới địa điểm"
            )

            # Nếu người dùng yêu cầu nhiều waypoint
            # thì đi điểm tiếp theo.
            QTimer.singleShot(
                300,
                lambda: self._send_next_waypoint(generation)
            )

        elif status == GoalStatus.STATUS_CANCELED:

            self.log(
                "[robot] Điều hướng đã bị hủy"
            )

            self._waypoint_queue.clear()

        else:

            self.log(
                f"[robot] Điều hướng thất bại, "
                f"status={status}"
            )

            self._waypoint_queue.clear()

    def _nav_feedback_callback(
        self,
        feedback_msg
    ):

        try:
            distance = (
                feedback_msg.feedback
                .distance_remaining
            )

            # Không log từng frame vì sẽ spam.
            # Nếu cần sau này có thể throttle.
            print(
                f"[NAV] Còn {distance:.2f} m"
            )

        except Exception:
            pass

    def cancel_navigation(self):
        self._navigation_generation += 1

        self._waypoint_queue.clear()

        if self._nav_goal_handle is None:
            self.log(
                'Đang chờ phản hồi mục tiêu để hủy.' if getattr(self, '_nav_pending', False)
                else 'Không có mục tiêu điều hướng đang chạy.'
            )
            return

        self.log(
            "[robot] Đang hủy navigation"
        )

        if not getattr(self, '_nav_cancel_requested', False):
            self._nav_cancel_requested = True
            self._nav_goal_handle.cancel_goal_async().add_done_callback(self._waypoint_cancel_response)

    def _waypoint_cancel_response(self, future):
        try:
            if not future.result().goals_canceling:
                self._nav_cancel_requested = False
                self.log('Nav2 chưa nhận hủy mục tiêu; chờ kết quả hoặc nhấn Dừng để thử lại.')
        except Exception as exc:
            self._nav_cancel_requested = False
            self.log(f'Lỗi hủy mục tiêu; chưa xác nhận dừng: {exc}')

    def _navigation_busy(self):
        return (self._nav_goal_handle is not None or getattr(self, '_nav_pending', False)
                or bool(self._waypoint_queue))

    def delete_waypoint(self, key):
        try:
            if self._navigation_busy():
                raise ValueError('Hãy dừng hành trình trước khi xóa địa điểm.')
            data, snapshot = load_waypoint_file(self._waypoints_file)
            reason = deletion_reason(key, data[key])
            if reason:
                raise ValueError(reason)
            refs = route_references(UI_DIRECTORY / 'multi_waypoints.json', key, data[key])
            if refs:
                raise ValueError('Địa điểm đang được sử dụng trong lộ trình: ' + ', '.join(refs))
            if QMessageBox.question(self, 'Xác nhận xóa', f'Xóa địa điểm {key}?') != QMessageBox.StandardButton.Yes:
                return False
            del data[key]
            self._waypoints, _ = save_waypoint_file(self._waypoints_file, data, snapshot)
            self._refresh_map_window()
            return True
        except (OSError, ValueError, KeyError) as exc:
            QMessageBox.warning(self, 'Không thể xóa', str(exc))
            return False

if __name__ == '__main__':
    rclpy.init()
    app = QApplication(sys.argv)
    app.setFont(QFont('Fira Sans',12))
    window = RobotUI()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    window.showMaximized()
    try:
        sys.exit(app.exec())
    finally:
        rclpy.shutdown()
