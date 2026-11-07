"""Curated v1.0; original retained in archive/. Launch via run.py."""
from software import settings
from software.settings import CONFIG, RUNTIME
import sys
import cv2
import numpy as np
import serial
import struct
import yaml
import csv
import datetime
import os
import traceback
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QPoint, QRect, QSize
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QFont, QIcon, QFontDatabase
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTextEdit, QLineEdit, QGroupBox, QRadioButton, QSlider, QGridLayout, QSizePolicy, QComboBox, QFileDialog)

# ---------------------------- 摄像头线程 ----------------------------
class CameraThread(QThread):
    frame_ready = pyqtSignal(int, np.ndarray)
    error_occurred = pyqtSignal(int, str)

    def __init__(self, cam_id=0):
        super().__init__()
        self.cam_id = cam_id
        self.running = True
        self.cap = None

    def run(self):
        try:
            self.cap = cv2.VideoCapture(CONFIG["camera_ids"]["arm" if self.cam_id == 0 else "chassis"])
            if not self.cap.isOpened():
                self.error_occurred.emit(self.cam_id, "无法打开摄像头")
                return

            while self.running:
                ret, frame = self.cap.read()
                if not ret:
                    self.error_occurred.emit(self.cam_id, "摄像头读取失败")
                    break

                self.frame_ready.emit(self.cam_id, cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        except Exception as e:
            self.error_occurred.emit(self.cam_id, str(e))
        finally:
            if self.cap:
                self.cap.release()
                self.cap = None

    def stop(self):
        self.running = False
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.wait(2000)

# ---------------------------- 视频显示组件 ----------------------------
class VideoLabel(QLabel):
    selection_completed = pyqtSignal(int, int, int, int)

    def __init__(self, cam_id=0):
        super().__init__()
        self.cam_id = cam_id
        self.setAlignment(Qt.AlignCenter)
        self.start_point = QPoint()
        self.end_point = QPoint()
        self.drawing = False
        self.max_area = QRect(50, 50, 740, 580)
        self.selected_area = QRect()
        self.setStyleSheet("background-color: #111; color: #b8c6d8;")
        self.setMinimumSize(320, 240)
        self.original_size = QSize(640, 480)
        self.setText("摄像头初始化中..." if cam_id == 0 else "底盘摄像头未连接")

    def update_pixmap(self, pixmap):
        super().setPixmap(pixmap)
        self.original_size = pixmap.size()

    def calculate_real_coords(self):
        if not self.pixmap():
            return
        img_w = self.original_size.width()
        img_h = self.original_size.height()
        label_w = self.width()
        label_h = self.height()
        scale = min(label_w / img_w, label_h / img_h)
        offset_x = (label_w - img_w * scale) / 2
        offset_y = (label_h - img_h * scale) / 2
        x1 = int((self.start_point.x() - offset_x) / scale)
        y1 = int((self.start_point.y() - offset_y) / scale)
        x2 = int((self.end_point.x() - offset_x) / scale)
        y2 = int((self.end_point.y() - offset_y) / scale)
        # 更严格的边界检查
        x1 = max(0, min(x1, img_w - 1))
        y1 = max(0, min(y1, img_h - 1))
        x2 = max(0, min(x2, img_w - 1))
        y2 = max(0, min(y2, img_h - 1))
        if x1 >= x2 or y1 >= y2:
            return
        self.selection_completed.emit(x1, y1, x2, y2)
        self.selected_area = QRect(x1, y1, abs(x2 - x1), abs(y2 - y1))
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        if not self.pixmap():
            return

        painter = QPainter(self)
        # 优化后的黄色框绘制
        painter.setPen(QPen(Qt.yellow, 2, Qt.SolidLine))  # 改为实线
        max_rect = self._scale_rect(self.max_area)
        painter.drawRect(max_rect)

        # 绘制已选区域
        if not self.selected_area.isNull():
            painter.setPen(QPen(Qt.green, 2, Qt.SolidLine))
            painter.drawRect(self._scale_rect(self.selected_area))

        # 绘制临时选框
        if self.drawing:
            painter.setPen(QPen(Qt.red, 2, Qt.DashLine))
            painter.drawRect(QRect(self.start_point, self.end_point))

    def _scale_rect(self, rect):
        """将原始图像坐标转换为显示坐标"""
        img_w = self.original_size.width()
        img_h = self.original_size.height()
        label_w = self.width()
        label_h = self.height()
        scale = min(label_w / img_w, label_h / img_h)
        offset_x = (label_w - img_w * scale) / 2
        offset_y = (label_h - img_h * scale) / 2
        x = int(rect.x() * scale + offset_x)
        y = int(rect.y() * scale + offset_y)
        width = int(rect.width() * scale)
        height = int(rect.height() * scale)
        # 新增边界检查，确保矩形不超出标签边界
        x = max(0, min(x, label_w - width))
        y = max(0, min(y, label_h - height))
        return QRect(x, y, width, height)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.start_point = event.pos()
            self.end_point = event.pos()
            self.drawing = True

    def mouseMoveEvent(self, event):
        if self.drawing and event.buttons() & Qt.LeftButton:
            self.end_point = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.drawing:
            self.end_point = event.pos()
            self.drawing = False
            self.calculate_real_coords()

# ---------------------------- 串口控制器 ----------------------------
class SprayController:
    def __init__(self):
        self.ser = None
        self.error_msg = ""
        self.is_connected = False
        self.connected_port = ""

    def connect(self, port):
        self.disconnect()  # 确保先断开已有连接
        try:
            if settings.OFFLINE:
                raise ValueError("离线模式不连接串口")
            self.ser = serial.Serial(port, CONFIG["serial_baudrate"], timeout=1)
            self.is_connected = True
            self.connected_port = port  # 记录连接的端口
            return True
        except serial.SerialException as e:
            self.error_msg = str(e)
            self.log_error(f"串口连接错误: {e}\n{traceback.format_exc()}")
            return False
        except Exception as e:
            self.error_msg = str(e)
            self.log_error(f"未知错误在串口连接时发生: {e}\n{traceback.format_exc()}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception as e:
                self.error_msg = str(e)
                self.log_error(f"串口关闭错误: {e}\n{traceback.format_exc()}")
        self.is_connected = False
        self.connected_port = ""  # 清除连接记录

    def send_command(self, area=0, speed=0, thickness=0, joints=None, movement=None):
        if not self.is_connected:
            raise ConnectionError("串口未连接")
        try:
            if joints is not None:  # 机械臂控制
                if not isinstance(joints, list) or len(joints) != 6:
                    raise ValueError("需要6个关节参数，且必须为列表类型")
                for joint in joints:
                    if not isinstance(joint, (int, float)):
                        raise ValueError("关节参数必须为数字类型")
                data = struct.pack('<6f', *joints)
                packet = bytes([0xAA, 0x01, len(data)]) + data
            elif movement is not None:  # 底盘控制
                if not isinstance(movement, (list, tuple)) or len(movement) != 3:
                    raise ValueError("需要3个运动参数，且必须为列表类型")
                for move in movement:
                    if not isinstance(move, (int, float)):
                        raise ValueError("运动参数必须为数字类型")
                data = struct.pack('<3f', *movement)
                packet = bytes([0xAA, 0x02, len(data)]) + data
            else:  # 喷涂参数
                if not isinstance(area, (int, float)) or not isinstance(speed, (int, float)) or not isinstance(
                        thickness, (int, float)):
                    raise ValueError("喷涂参数必须为数字类型")
                data = struct.pack('<fHf', area, speed, thickness)
                packet = bytes([0xAA, 0x03, len(data)]) + data
            packet += bytes([sum(packet) & 0xFF, 0x55])
            # 检查串口写入是否成功
            written_bytes = self.ser.write(packet)
            if written_bytes != len(packet):
                raise serial.SerialException(f"串口写入失败，期望写入 {len(packet)} 字节，实际写入 {written_bytes} 字节")
        except struct.error as se:
            print(f"数据打包错误: {str(se)}")
            self.log_error(f"数据打包错误: {str(se)}")
            raise
        except ValueError as ve:
            print(f"参数错误: {str(ve)}")
            self.log_error(f"参数错误: {str(ve)}")
            raise
        except serial.SerialException as sse:
            print(f"串口操作错误: {str(sse)}")
            self.log_error(f"串口操作错误: {str(sse)}")
            raise
        except Exception as e:
            print(f"发送指令失败: {str(e)}")
            self.log_error(f"发送指令失败: {str(e)}")
            raise

    def log_error(self, error_msg):
        print(error_msg)

# -------------------- 坐标转换核心算法 --------------------
def pixel_to_world_plane(points_pixel, camera_matrix, T_cam_to_base, plane_z=0, dist_coeffs=None):
    try:
        points = np.array(points_pixel, dtype=np.float32).reshape(-1, 1, 2)
        if dist_coeffs is None:
            dist_coeffs = np.zeros((4,))
        points_undist = cv2.undistortPoints(points, camera_matrix, dist_coeffs, None, camera_matrix)
        if points_undist is None:
            return None
        # 转换为齐次坐标
        rays = cv2.convertPointsToHomogeneous(points_undist).reshape(-1, 3)
        if rays is None:
            return None
        # 计算射线方向（相机坐标系）
        rays_cam = (np.linalg.inv(camera_matrix) @ rays.T).T
        # 转换到基座坐标系
        R = T_cam_to_base[:3, :3]
        t = T_cam_to_base[:3, 3]
        rays_base = (R @ rays_cam.T).T
        # 计算与平面交点
        denominator = rays_base[:, 2]
        valid_mask = np.abs(denominator) > 1e-6
        if not np.any(valid_mask):
            return None
        # 仅处理有效射线
        scale = (plane_z - t[2]) / denominator[valid_mask]
        points_3d = t + rays_base[valid_mask] * scale[:, np.newaxis]
        return points_3d
    except Exception as e:
        print(f"平面转换错误: {str(e)}")
        return None

def pixel_to_cylinder(points_pixel, camera_matrix, T_cam_to_base, cylinder_radius=0.5, dist_coeffs=None):
    try:
        points_cam = cv2.undistortPoints(np.array(points_pixel, dtype=np.float32),
                                         camera_matrix, dist_coeffs)
        print("After undistortPoints, points_cam:", points_cam)
        points_cam = cv2.convertPointsToHomogeneous(points_cam).reshape(-1, 3)
        print("After convertPointsToHomogeneous, points_cam:", points_cam)
        points_base = (T_cam_to_base[:3, :3] @ points_cam.T).T + T_cam_to_base[:3, 3]
        print("points_base before calculating theta and z:", points_base)
        theta = np.arctan2(points_base[:, 1], points_base[:, 0])
        z = points_base[:, 2]
        print("z coordinates:", z)
        width = cylinder_radius * (np.max(theta) - np.min(theta))
        height = np.max(z) - np.min(z)
        return width * height
    except Exception as e:
        print(f"圆柱转换错误: {str(e)}")
        return 0

def calculate_polygon_area(points):
    try:
        pts = np.array(points, dtype=np.float64)
        if len(pts) < 3:
            return 0.0
        x = pts[:, 0]
        y = pts[:, 1]
        return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))
    except Exception as e:
        print(f"[DEBUG] 面积计算错误: {str(e)}")
        return 0.0

def resource_path(relative_path):
    """ 获取资源的绝对路径 """
    if hasattr(sys, '_MEIPASS'):
        # 如果是打包后的可执行文件，使用临时目录路径
        base_path = sys._MEIPASS
    else:
        # 如果是开发环境，使用当前目录路径
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)

# -------------------- 主界面 --------------------
class MainWindow(QMainWindow):
    log_signal = pyqtSignal(str)  # 新增信号

    def __init__(self):
        # 调用父类的构造函数
        super().__init__()
        # 设置窗口标题和几何形状
        self.setWindowTitle("喷漆机器人控制平台 v1.0 · 整理版")
        self.setGeometry(100, 100, 1600, 900)
        # 初始化相关属性
        self.calibrating = False
        self.calibration_images = []
        self.current_area = 0.0
        self.distance = 0.0
        self.joint_angles = [0.0] * 6  # 六个关节角度
        self.calib_file = str(RUNTIME / "camera_calibration.yaml")
        self.calibration_loaded = False
        # 初始化相机矩阵、畸变系数等
        self.camera_matrix = np.array([
            [1200, 0, 640],
            [0, 1200, 360],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objp = np.zeros((6 * 7, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
        self.T_cam_to_base = np.eye(4)
        # 初始化日志和错误日志组件
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.error_log = QTextEdit()
        self.error_log.setReadOnly(True)
        self.error_log.setStyleSheet("color: red;")  # 红色突出错误
        # 初始化控制器
        self.controller = SprayController()
        # 初始化相机和加载标定数据
        self.cylinder_radius = 0.5  # 默认圆柱半径
        # 连接信号
        self.log_signal.connect(self.update_log_safely)
        # 初始化用户界面
        self.init_ui()
        self.cam_threads = []
        self.load_calibration()
        if not settings.OFFLINE:
            self.init_camera()
        else:
            self.video_arm.setText("离线预览：未连接机械臂摄像头")
            self.video_chassis.setText("离线预览：未连接底盘摄像头")

    def load_default_calibration(self):
        # 这里加载默认标定数据，你可以根据实际情况修改
        default_data = {
            "camera_matrix": np.eye(3),
            "dist_coeffs": np.zeros(5),
            "T_cam_to_base": np.eye(4)
        }
        return default_data

    def update_log_safely(self, message):
        self.log.append(message)

    def init_ui(self):
        main_widget = QWidget()
        main_layout = QHBoxLayout()
        # 左侧视频区域
        video_layout = QVBoxLayout()
        self.video_arm = VideoLabel(0)
        self.video_chassis = VideoLabel(1)
        video_layout.addWidget(self.video_arm)
        video_layout.addWidget(self.video_chassis)
        # 右侧控制面板
        control_layout = QVBoxLayout()
        # 定义喷涂参数输入框
        self.speed_input = QLineEdit("1000")
        self.thickness_input = QLineEdit("0.2")
        # 参数组
        param_group = QGroupBox("喷涂参数")
        param_layout = QVBoxLayout()
        param_layout.addWidget(QLabel("速度 (RPM):"))
        param_layout.addWidget(self.speed_input)
        param_layout.addWidget(QLabel("厚度 (mm):"))
        param_layout.addWidget(self.thickness_input)
        param_group.setLayout(param_layout)
        # 使用 resource_path 函数获取图标文件的绝对路径
        icon_connect_path = resource_path(os.path.join('icons', '连接串口.ico'))
        icon_start_path = resource_path(os.path.join('icons', '启动喷涂.ico'))
        icon_stop_path = resource_path(os.path.join('icons', '紧急停止.ico'))
        icon_save_calib_path = resource_path(os.path.join('icons', '保存.ico'))
        icon_load_calib_path = resource_path(os.path.join('icons', '加载.ico'))
        icon_export_log_path = resource_path(os.path.join('icons', '导出.ico'))
        # 创建 QIcon 对象
        icon_connect = QIcon(icon_connect_path)
        icon_start = QIcon(icon_start_path)
        icon_stop = QIcon(icon_stop_path)
        icon_save_calib = QIcon(icon_save_calib_path)
        icon_load_calib = QIcon(icon_load_calib_path)
        icon_export_log = QIcon(icon_export_log_path)
        # 控制按钮
        self.btn_connect = QPushButton("连接串口")
        btn_connect = self.btn_connect
        btn_connect.setIcon(icon_connect)  # 为连接串口按钮设置图标
        btn_start = QPushButton("启动喷涂")
        btn_start.setIcon(icon_start)  # 为启动喷涂按钮设置图标
        btn_stop = QPushButton("紧急停止")
        btn_stop.setIcon(icon_stop)  # 为紧急停止按钮设置图标
        btn_save_calib = QPushButton("保存标定参数")
        btn_save_calib.setIcon(icon_save_calib)  # 为保存标定参数按钮设置图标
        btn_load_calib = QPushButton("加载标定图片")
        btn_load_calib.setIcon(icon_load_calib)  # 为加载标定图片按钮设置图标
        btn_export_log = QPushButton("导出日志")
        btn_export_log.setIcon(icon_export_log)  # 为导出日志按钮设置图标
        # 新增：为按钮添加信号与槽的连接
        btn_connect.clicked.connect(self.connect_serial)
        btn_start.clicked.connect(self.start_spraying)
        btn_stop.clicked.connect(self.emergency_stop)
        btn_save_calib.clicked.connect(self.save_calibration)
        btn_load_calib.clicked.connect(self.load_calibration_images)
        btn_export_log.clicked.connect(self.export_log)
        # 按钮布局
        btn_layout = QHBoxLayout()  # 确保 btn_layout 被正确初始化
        btn_layout.addWidget(btn_connect)
        btn_layout.addWidget(btn_start)
        btn_layout.addWidget(btn_stop)
        btn_layout.addWidget(btn_save_calib)
        btn_layout.addWidget(btn_load_calib)
        btn_layout.addWidget(btn_export_log)
        # 串口选择组件
        serial_group = QGroupBox("串口连接")
        serial_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        serial_layout.addWidget(self.port_combo, 70)
        serial_layout.addWidget(self.btn_refresh, 30)
        serial_group.setLayout(serial_layout)
        control_layout.insertWidget(0, serial_group)  # 放在控制面板最上方
        # 连接刷新按钮
        self.serial_timer = QTimer()
        self.serial_timer.timeout.connect(self.refresh_ports)
        self.serial_timer.start(2000)  # 每2秒刷新一次
        # 添加模式选择单选按钮
        mode_group = QGroupBox("计算模式")
        mode_layout = QVBoxLayout()
        self.radio_plane = QRadioButton("平面模式")
        self.radio_cylinder = QRadioButton("圆柱模式")
        self.radio_plane.setChecked(True)
        # 默认选中平面模式
        mode_layout.addWidget(self.radio_plane)
        mode_layout.addWidget(self.radio_cylinder)
        mode_group.setLayout(mode_layout)
        control_layout.addWidget(mode_group)
        control_layout.addWidget(param_group)
        control_layout.addLayout(btn_layout)
        # 添加日志显示组件
        self.log.setReadOnly(True)
        self.log.setMaximumHeight(80)
        control_layout.addWidget(self.log)
        # 状态显示
        status_group = QGroupBox("系统状态")
        status_layout = QVBoxLayout()
        self.distance_label = QLabel("距离: -- m")
        self.joint_status = QLabel("关节角度: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
        self.spray_result = QLabel("喷涂面积: -- m² | 坐标: --")  # 新增结果显示
        status_layout.addWidget(self.distance_label)
        status_layout.addWidget(self.joint_status)
        status_layout.addWidget(self.spray_result)
        status_group.setLayout(status_layout)# 添加结果显示
        # 在init_ui方法中，在控制面板添加：
        status_layout.addWidget(QLabel("错误日志:"))
        self.error_log.setMaximumHeight(45)
        status_layout.addWidget(self.error_log)
        # 机械臂控制
        arm_group = QGroupBox("六自由度机械臂控制")
        arm_layout = QGridLayout()
        self.joint_sliders = []
        for i in range(6):
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 180)
            slider.valueChanged.connect(lambda val, idx=i: self.update_joint(idx, val))
            arm_layout.addWidget(QLabel(f"关节{i + 1}:"), i, 0)
            arm_layout.addWidget(slider, i, 1)
            self.joint_sliders.append(slider)
        arm_group.setLayout(arm_layout)
        # 底盘控制
        move_group = QGroupBox("底盘控制")
        move_layout = QGridLayout()
        self.btn_forward = QPushButton("↑")
        self.btn_back = QPushButton("↓")
        self.btn_left = QPushButton("←")
        self.btn_right = QPushButton("→")
        move_layout.addWidget(self.btn_forward, 0, 1)
        move_layout.addWidget(self.btn_back, 2, 1)
        move_layout.addWidget(self.btn_left, 1, 0)
        move_layout.addWidget(self.btn_right, 1, 2)
        move_group.setLayout(move_layout)
        self.cylinder_radius_input = QLineEdit("0.5")
        param_layout.addWidget(QLabel("圆柱半径 (m):"))
        param_layout.addWidget(self.cylinder_radius_input)
        # 组装控制面板
        control_layout.addWidget(status_group)
        control_layout.addWidget(arm_group)
        control_layout.addWidget(move_group)
        control_layout.addStretch()
        # 合并布局
        main_layout.addLayout(video_layout, 60)
        main_layout.addLayout(control_layout, 40)
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)
        # 连接信号
        self.video_arm.selection_completed.connect(lambda x1, y1, x2, y2: self.handle_selection(x1, y1, x2, y2, 0))
        self.video_chassis.selection_completed.connect(lambda x1, y1, x2, y2: self.handle_selection(x1, y1, x2, y2, 1))
        self.btn_forward.clicked.connect(lambda: self.move_chassis('forward'))
        self.btn_back.clicked.connect(lambda: self.move_chassis('backward'))
        self.btn_left.clicked.connect(lambda: self.move_chassis('left'))
        self.btn_right.clicked.connect(lambda: self.move_chassis('right'))
        # 定时更新
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(200)

    def init_camera(self):
        self.cam_threads = []
        try:
            for cam_id in [0, 1]:
                thread = CameraThread(cam_id)
                thread.frame_ready.connect(self.update_frame)
                thread.error_occurred.connect(self.handle_cam_error)
                thread.start()
                self.cam_threads.append(thread)
        except Exception as e:
            self.log_error(f"摄像头初始化失败: {str(e)}")
        self.joint_limits = [
            (0, 180), (0, 180), (0, 180),
            (0, 180), (0, 180), (0, 180)
        ]
        self.chassis_speed = 0.5  # m/s

    def update_joint(self, joint_idx, value):
        self.joint_angles[joint_idx] = value
        self.joint_status.setText(f"关节指令角度: {self.joint_angles}")
        if self.controller.is_connected:
            try:
                self.controller.send_command(joints=self.joint_angles)
            except Exception as exc:
                self.log.append(f"关节指令失败：{exc}")


    def move_chassis(self, direction):
        if not self.controller.is_connected:
            self.log.append("串口未连接，未下发底盘指令")
            return
        speed_dict = {'forward': (0.5, 0, 0), 'backward': (-0.5, 0, 0),
                      'left': (0, -0.5, 0), 'right': (0, 0.5, 0)}
        try:
            self.controller.send_command(movement=speed_dict[direction])
        except Exception as exc:
            self.log.append(f"底盘指令失败：{exc}")


    def update_frame(self, cam_id, frame):
        target_label = self.video_arm if cam_id == 0 else self.video_chassis
        try:
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_img).scaled(
                target_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            target_label.update_pixmap(pixmap)
            target_label.original_size = QSize(w, h)

            if self.calibrating:
                gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
                ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)
                if ret:
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                    img = cv2.drawChessboardCorners(frame.copy(), (7, 6), corners2, ret)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    q_img = QImage(img.data, img.shape[1], img.shape[0], img.strides[0], QImage.Format_RGB888)
                    pixmap = QPixmap.fromImage(q_img).scaled(
                        target_label.width(), target_label.height(),
                        Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    target_label.update_pixmap(pixmap)
                    self.calibration_images.append(gray)
                    self.log.append(f"已捕获第 {len(self.calibration_images)} 张有效标定图像。")
                    if len(self.calibration_images) >= 10:
                        self.calibrating = False
                        self.perform_calibration()
        except Exception as e:
            self.log_error(f"画面更新错误: {str(e)}")

    def start_calibration(self):
        self.calibrating = True
        self.log.append("开始摄像头标定，请将棋盘格置于摄像头视野内。")

    def handle_cam_error(self, cam_id, msg):
        self.log_signal.emit(f"摄像头 {cam_id} 错误: {msg}")

    def handle_selection(self, x1, y1, x2, y2, cam_id):
        log_message = ""  # 初始化log_message
        self.current_area = 0.0
        self.spray_result.setText("喷涂面积: -- m²")
        if not self.calibration_loaded:
            # 提示用户先选择标定文件
            self.log_error("请先选择标定文件")
            return
        try:
            label = self.video_arm if cam_id == 0 else self.video_chassis
            pixmap = label.pixmap()
            if not pixmap or pixmap.isNull():
                raise ValueError("摄像头画面未加载，无法框选")
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(label.original_size.width(), x2), min(label.original_size.height(), y2)
            if x1 >= x2 or y1 >= y2:
                raise ValueError("无效的框选区域")
            # 检查相关矩阵和系数
            if self.camera_matrix is None or self.camera_matrix.shape != (3, 3):
                raise ValueError("无效的相机矩阵")
            if self.dist_coeffs is None or self.dist_coeffs.shape[0] < 4:
                raise ValueError("无效的畸变系数")
            if self.T_cam_to_base is None or self.T_cam_to_base.shape != (4, 4):
                raise ValueError("无效的相机到基座变换矩阵")
            points_pixel = [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
            if self.radio_plane.isChecked():
                world_points = pixel_to_world_plane(
                    points_pixel,
                    self.camera_matrix,
                    self.T_cam_to_base,
                    plane_z=0,
                    dist_coeffs=self.dist_coeffs
                )
                # 调试输出
                print("转换后的世界坐标:\n", world_points)
                self.log.append(f"世界坐标样本: {world_points[0] if world_points is not None else 'None'}")

                if world_points is None:
                    raise ValueError("坐标转换失败，请检查标定参数和摄像头位置")
            # 确保点集形成闭合多边形
                closed_points = np.vstack([world_points[:, :2], world_points[0, :2]])
                area = calculate_polygon_area(closed_points)
            else:
                # 从输入框获取半径并验证
                try:
                    self.cylinder_radius = float(self.cylinder_radius_input.text())
                    if self.cylinder_radius <= 0:
                        raise ValueError("圆柱半径必须大于0")
                except ValueError as ve:
                    self.log.append(f"参数错误: {str(ve)}")
                    return

                area = pixel_to_cylinder(
                    points_pixel,
                    self.camera_matrix,
                    self.T_cam_to_base,
                    self.cylinder_radius,
                    dist_coeffs=self.dist_coeffs
                )

                if area is None:
                    raise ValueError("圆柱坐标转换失败，请检查参数")
                    # 更新当前面积
            if area is not None:
                self.current_area = max(area, 0.0)  # 确保非负
                self.spray_result.setText(f"喷涂面积: {self.current_area:.4f} m²")
            else:
                self.log.append("面积计算失败，请检查相关函数返回值")
        except Exception as e:
            self.log.append(f"处理选择区域时出错: {str(e)}")



    def update_status(self):
        self.distance_label.setText("距离: --（未接入距离反馈）")


    def refresh_ports(self):
        selected = self.port_combo.currentText()
        entries = [] if settings.OFFLINE else [f"{p.device} - {p.description}" for p in serial.tools.list_ports.comports()]
        self.port_combo.clear()
        self.port_combo.addItems(entries)
        index = self.port_combo.findText(selected)
        if index >= 0:
            self.port_combo.setCurrentIndex(index)


    def closeEvent(self, event):
        self.timer.stop()
        self.serial_timer.stop()
        for thread in self.cam_threads:
            thread.stop()
            thread.wait()
        self.controller.disconnect()
        event.accept()


    def log_error(self, error_msg):
        print(error_msg)

    def log_to_csv(self, data):
        try:
            with open('spraying_log.csv', 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(data)
        except Exception as e:
            self.log_error(f"日志记录错误: {str(e)}")

    def is_port_in_use(port):
        try:
            s = serial.Serial(port)
            s.close()
            return False
        except serial.SerialException:
            return True

    def connect_serial(self):
        try:
            if not self.port_combo.count():
                self.log.append("没有可用串口")
                return

            selected = self.port_combo.currentText().split(" - ")[0]

            if self.controller.is_connected:
                self.controller.disconnect()
                self.log.append(f"已断开 {selected}")
                self.btn_connect.setText("连接串口")  # 重置按钮文本
                return

            # 添加端口可用性检查
            available_ports = [p.device for p in serial.tools.list_ports.comports()]
            if selected not in available_ports:
                self.log.append(f"端口 {selected} 已不存在")
                return

            # 添加连接超时处理
            if self.controller.connect(selected):
                self.log.append(f"成功连接 {selected}")
                self.btn_connect.setText(f"断开 {selected}")
            else:
                self.log.append(f"连接失败: {self.controller.error_msg}")
                self.btn_connect.setText("连接串口")  # 连接失败时重置按钮

        except Exception as e:
            error_msg = f"连接异常: {str(e)}\n{traceback.format_exc()}"
            self.log.append(error_msg)
            self.btn_connect.setText("连接串口")  # 异常时重置按钮
            print(error_msg)

    def start_spraying(self):
        if not self.controller.is_connected:
            self.log.append("未连接串口，请先连接串口。")
            return
        try:
            # 添加参数有效性验证
            speed = float(self.speed_input.text())
            thickness = float(self.thickness_input.text())
            if not np.isfinite([speed, thickness]).all() or not 1 <= speed <= 65535 or thickness <= 0:
                raise ValueError("参数必须大于零")

            # 添加面积有效性检查
            if not np.isfinite(self.current_area) or self.current_area <= 0:
                raise ValueError("无效喷涂区域")

            self.controller.send_command(
                area=self.current_area,
                speed=int(speed),
                thickness=thickness
            )
            self.log.append(
                f"启动喷涂，面积: {self.current_area:.4f} 平方米，"
                f"速度: {speed} RPM，厚度: {thickness} mm"
            )
            self.log_to_csv([self.current_area, speed, thickness])

        except ValueError as ve:
            self.log.append(f"参数错误: {str(ve)}")
        except Exception as e:
            self.log.append(f"喷涂失败: {str(e)}")

    def emergency_stop(self):
        self.controller.disconnect()
        self.log.append("已断开串口；下位机停止动作需由其固件实现，请使用独立停止控制。")


    def save_calibration(self):
        if not self.calibration_loaded:
            self.log.append("尚无实测外参，不保存默认矩阵。相机内参标定不等同于手眼标定。")
            return
        data = {'camera_matrix': self.camera_matrix.tolist(), 'dist_coeffs': self.dist_coeffs.tolist(),
                'T_cam_to_base': self.T_cam_to_base.tolist()}
        try:
            with open(self.calib_file, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(data, stream)
            self.log.append("标定参数已保存")
        except OSError as exc:
            self.log.append(f"保存失败：{exc}")


    def load_calibration(self):
        self.calibration_loaded = False
        if not os.path.exists(self.calib_file):
            self.log.append("缺少实测标定文件；面积计算暂不可用。请参考 docs/software.md。")
            return
        try:
            with open(self.calib_file, encoding="utf-8") as stream:
                data = yaml.safe_load(stream)
            matrix = np.asarray(data['camera_matrix'], dtype=float).reshape(3, 3)
            distortion = np.asarray(data['dist_coeffs'], dtype=float).flatten()
            transform = np.asarray(data['T_cam_to_base'], dtype=float).reshape(4, 4)
            if not all(np.isfinite(v).all() for v in (matrix, distortion, transform)) or distortion.size < 4:
                raise ValueError("标定包含无效数值")
            if abs(np.linalg.det(matrix)) < 1e-12 or not np.allclose(transform[3], [0, 0, 0, 1]):
                raise ValueError("标定矩阵无效")
            self.camera_matrix, self.dist_coeffs, self.T_cam_to_base = matrix, distortion, transform
            self.calibration_loaded = True
            self.log.append("已加载实测标定文件")
        except Exception as exc:
            self.log.append(f"标定加载失败：{exc}")


    def create_default_calibration(self):
        self.calibration_loaded = False
        self.log.append("需要实测标定；不自动生成实验替代数据。")


    def load_calibration_images(self):
        folder = QFileDialog.getExistingDirectory(self, "选择标定图片文件夹")
        if not folder: return
        self.calibration_images = []
        valid_count = 0
        for fname in sorted(os.listdir(folder)):
            if fname.lower().endswith(('.jpg', '.jpeg', '.png')):
                path = os.path.join(folder, fname)
                img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if img is None: continue

                # 使用动态棋盘格检测
                for grid_size in [(7, 6), (9, 6)]:
                    ret, corners = cv2.findChessboardCorners(img, grid_size)
                    if ret:
                        corners2 = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1),
                                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                        self.calibration_images.append((img, grid_size))
                        valid_count += 1
                        break
        if valid_count >= 10:
            self.perform_calibration()
        else:
            self.log.append(f"有效图片不足 ({valid_count}/10)")

    def perform_calibration(self):
        """执行标定计算"""
        try:
            objpoints = []
            imgpoints = []

            # 生成对应棋盘格的对象点
            for img, (w, h) in self.calibration_images:
                objp = np.zeros((w * h, 3), np.float32)
                objp[:, :2] = np.mgrid[0:w, 0:h].T.reshape(-1, 2)

                ret, corners = cv2.findChessboardCorners(img, (w, h))
                if ret:
                    corners2 = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1),
                                                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    objpoints.append(objp)
                    imgpoints.append(corners2)
            # 执行标定
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, self.calibration_images[0][0].shape[::-1], None, None)
            if ret:
                # 更新标定参数
                self.camera_matrix = mtx
                self.dist_coeffs = dist
                self.save_calibration()
                self.log.append("相机内参计算完成；外参仍需独立实测。")
            else:
                self.log.append("标定失败！")
        except Exception as e:
            self.error_log.append(f"标定错误: {traceback.format_exc()}")

    def calculate_reprojection_error(self, objpoints, imgpoints, mtx, dist, rvecs, tvecs):
        total_error = 0
        for i in range(len(objpoints)):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
            total_error += error
        return total_error / len(objpoints)


    def log_to_csv(self, data):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            with open(RUNTIME / 'spray_log.csv', 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, *data])
        except Exception as e:
            self.log_error(f"日志记录失败: {str(e)}")

    def log_error(self, error_msg):
        print(error_msg)
        self.log.append(f"错误: {error_msg}")

    def export_log(self):
        path, _ = QFileDialog.getSaveFileName(self, "导出日志", "", "CSV文件 (*.csv)")
        if path:
            try:
                with open(RUNTIME / 'spray_log.csv', 'r') as src, open(path, 'w') as dst:
                    dst.write(src.read())
                self.log.append(f"日志已导出到 {path}")
            except Exception as e:
                self.log.append(f"导出失败: {str(e)}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
