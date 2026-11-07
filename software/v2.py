"""Curated v2.0; original retained in archive/. Launch via run.py."""
from software import settings
from software.settings import CONFIG
from software.planning import rectangle_path, circle_path, map_2d, project_path
from software.execution import TrajectoryExecutor, Outcome
from threading import Lock
import sys
import cv2
import numpy as np
from PySide6.QtWidgets import *
from PySide6.QtCore import *
from PySide6.QtGui import *
import time
try:
    from xarm.wrapper import XArmAPI
except ImportError:
    XArmAPI = None
import math
try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


# ======================
# RealSense 双目摄像头管理类
# ======================
class RealSenseCamera:
    def __init__(self):
        self.pipeline = None
        self.config = None
        self.align = None
        self.depth_scale = None
        self.camera_available = False
        self.camera_info = {}

    def init_camera(self, width=640, height=480, fps=30):
        """初始化 RealSense 双目摄像头"""
        try:
            # 创建管道和配置
            self.pipeline = rs.pipeline()
            self.config = rs.config()

            # 启用 RGB 和深度流
            self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

            # 启动流
            profile = self.pipeline.start(self.config)

            # 创建对齐对象（对齐深度图到彩色图）
            self.align = rs.align(rs.stream.color)

            # 获取深度传感器缩放因子（mm 到 米）
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale() * 1000  # 转换为 mm

            # 获取相机内参
            color_profile = profile.get_stream(rs.stream.color)
            intr = color_profile.as_video_stream_profile().get_intrinsics()
            self.camera_info = {
                'fx': intr.fx,  # 焦距 X
                'fy': intr.fy,  # 焦距 Y
                'cx': intr.ppx,  # 主点 X
                'cy': intr.ppy,  # 主点 Y
                'width': width,
                'height': height
            }

            self.camera_available = True
            print(f" RealSense 相机初始化成功")
            print(f"   内参: fx={self.camera_info['fx']:.2f}, fy={self.camera_info['fy']:.2f}")
            return True

        except Exception as e:
            print(f" RealSense 相机初始化失败: {e}")
            self.camera_available = False
            return False

    def get_frames(self):
        """获取彩色图和深度图"""
        if not self.camera_available:
            return None, None, None

        try:
            # 等待帧数据
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)

            # 对齐深度图到彩色图
            aligned_frames = self.align.process(frames)

            # 获取彩色帧和深度帧
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            if not color_frame or not depth_frame:
                return None, None, None

            # 转换为 numpy 数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            return color_image, depth_image, depth_frame

        except Exception as e:
            print(f"获取帧失败: {e}")
            return None, None, None

    def get_depth_at_pixel(self, x, y, depth_frame):
        """获取指定像素的深度值（mm）"""
        if depth_frame is None:
            return -1
        try:
            depth = depth_frame.get_distance(x, y) * 1000  # 转换为 mm
            return depth
        except:
            return -1

    def get_3d_coordinate(self, x, y, depth_mm):
        """将像素坐标和深度转换为相机坐标系下的3D坐标（mm）"""
        if depth_mm <= 0:
            return None

        fx = self.camera_info['fx']
        fy = self.camera_info['fy']
        cx = self.camera_info['cx']
        cy = self.camera_info['cy']

        z = depth_mm
        x_cam = (x - cx) * z / fx
        y_cam = (y - cy) * z / fy

        return x_cam, y_cam, z

    def stop(self):
        if self.pipeline:
            try:
                self.pipeline.stop()
            except RuntimeError:
                pass
        self.camera_available = False



# ======================
# 视觉线程（RealSense 双目摄像头）
# ======================
class CameraThread(QThread):
    frame_signal = Signal(np.ndarray)
    rect_signal = Signal(float, float, float, float)
    circle_signal = Signal(float, float, float)
    trajectory_signal = Signal(list)
    camera_status_signal = Signal(bool)
    depth_signal = Signal(float, float, float)  # 深度信息信号

    def __init__(self):
        super().__init__()
        self.running = True
        self.rs_camera = None
        self.camera_available = False

        # 相机参数
        self.frame_width = 640
        self.frame_height = 480

        # 3D 坐标相关
        self.current_depth = 0
        self.show_depth_info = True

        # 标定数据
        self.paint_mode = "plane"
        self.paint_area_range = tuple(CONFIG["paint_bounds_px"])

        self.rect_x0, self.rect_y0 = -1, -1
        self.rect_x1, self.rect_y1 = -1, -1
        self.rect_confirmed = False
        self.rect_depth = 0  # 矩形区域的平均深度

        self.cx, self.cy = -1, -1
        self.radius = 0
        self.circle_confirmed = False
        self.circle_depth = 0  # 圆心深度

        # 轨迹
        self.trajectory_points = []
        self.trajectory_3d_points = []  # 3D 轨迹点
        self.show_trajectory = False

        # 深度采样点数（用于计算区域平均深度）
        self.depth_sample_points = []
        self.frame_lock = Lock()
        self.latest_depth = None
        self.latest_time = 0.0

        # 坐标转换参数（相机到机械臂）
        self.cam_to_arm_offset_x = CONFIG["mapping_3d"]["offset_mm"][0]
        self.cam_to_arm_offset_y = CONFIG["mapping_3d"]["offset_mm"][1]
        self.cam_to_arm_offset_z = CONFIG["mapping_3d"]["offset_mm"][2]

    def init_camera(self):
        if settings.OFFLINE or rs is None:
            self.camera_status_signal.emit(False)
            return False
        self.rs_camera = RealSenseCamera()
        self.camera_available = self.rs_camera.init_camera(self.frame_width, self.frame_height, 30)
        self.camera_status_signal.emit(self.camera_available)
        return self.camera_available


    def get_region_depth(self, x0, y0, x1, y1, depth_frame):
        """计算矩形区域的平均深度"""
        if depth_frame is None:
            return -1

        x_min = max(0, int(min(x0, x1)))
        x_max = min(self.frame_width, int(max(x0, x1)))
        y_min = max(0, int(min(y0, y1)))
        y_max = min(self.frame_height, int(max(y0, y1)))

        if x_min >= x_max or y_min >= y_max:
            return -1

        depths = []
        step = max(1, min((x_max - x_min), (y_max - y_min)) // 10)

        for y in range(y_min, y_max, step):
            for x in range(x_min, x_max, step):
                depth = self.rs_camera.get_depth_at_pixel(x, y, depth_frame)
                if depth > 0 and depth < 2000:  # 过滤无效深度（2米内有效）
                    depths.append(depth)

        if depths:
            return np.mean(depths)
        return -1

    def get_point_3d(self, x, y, depth_frame):
        """获取图像点的3D坐标"""
        depth = self.rs_camera.get_depth_at_pixel(x, y, depth_frame)
        if depth <= 0 or depth > 2000:
            return None

        # 转换为相机坐标系下的3D坐标
        x_cam, y_cam, z_cam = self.rs_camera.get_3d_coordinate(x, y, depth)

        if x_cam is None:
            return None

        # 转换到机械臂坐标系
        arm_x = 300 + x_cam * 0.5 + self.cam_to_arm_offset_x
        arm_y = 0 - y_cam * 0.5 + self.cam_to_arm_offset_y
        arm_z = z_cam + self.cam_to_arm_offset_z

        return arm_x, arm_y, arm_z

    def generate_plane_trajectory_3d(self, x0, y0, x1, y1, depth_frame, step=20):
        if depth_frame is None or not self.rs_camera:
            raise ValueError("缺少有效深度帧，请重新采集")
        return project_path(rectangle_path(x0, y0, x1, y1, step),
            lambda x, y: self.rs_camera.get_depth_at_pixel(x, y, depth_frame),
            self.rs_camera.camera_info, CONFIG['mapping_3d'], CONFIG['max_depth_mm'])


    def generate_circle_trajectory_3d(self, cx, cy, radius, depth_frame, step=15):
        if depth_frame is None or not self.rs_camera:
            raise ValueError("缺少有效深度帧，请重新采集")
        return project_path(circle_path(cx, cy, radius, step),
            lambda x, y: self.rs_camera.get_depth_at_pixel(x, y, depth_frame),
            self.rs_camera.camera_info, CONFIG['mapping_3d'], CONFIG['max_depth_mm'])


    def generate_plane_trajectory(self, x0, y0, x1, y1, step=20):
        return rectangle_path(x0, y0, x1, y1, step)


    def generate_circle_trajectory(self, cx, cy, radius, step=15):
        return circle_path(cx, cy, radius, step)


    def set_trajectory_and_show(self, points):
        self.trajectory_points = points
        self.show_trajectory = True

    def clear_trajectory(self):
        self.trajectory_points = []
        self.trajectory_3d_points = []
        self.show_trajectory = False

    def draw_depth_info(self, frame, x, y, depth):
        """在图像上绘制深度信息"""
        if depth > 0:
            cv2.putText(frame, f"Depth: {depth:.1f}mm", (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    def run(self):
        # 初始化 RealSense 摄像头
        if not self.init_camera():
            print("RealSense 摄像头未连接，显示无设备预览")

        last_depth_frame = None

        while self.running:
            if self.camera_available:
                color_frame, depth_frame, raw_depth_frame = self.rs_camera.get_frames()
                last_depth_frame = raw_depth_frame
                with self.frame_lock:
                    self.latest_depth = raw_depth_frame if color_frame is not None else None
                    self.latest_time = time.monotonic()

                if color_frame is not None:
                    frame = color_frame.copy()
                else:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(frame, "RealSense Camera Error", (200, 240),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 200, 255), 2)
            else:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "No RealSense Camera Detected", (150, 220),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (80, 200, 255), 2)
                cv2.putText(frame, "Please Connect Intel RealSense", (170, 260),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 200, 255), 2)

            # 绘制喷涂区域
            rx0, ry0, rx1, ry1 = self.paint_area_range
            cv2.rectangle(frame, (rx0, ry0), (rx1, ry1), (0, 255, 255), 3)
            cv2.putText(frame, "Paint Area", (rx0 + 10, ry0 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 显示相机信息
            if self.camera_available:
                cv2.putText(frame, "RealSense RGB-D", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(frame, "3D Mode Active", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # 绘制轨迹
            if self.show_trajectory and len(self.trajectory_points) > 1:
                for i in range(len(self.trajectory_points) - 1):
                    pt1 = (int(self.trajectory_points[i][0]), int(self.trajectory_points[i][1]))
                    pt2 = (int(self.trajectory_points[i + 1][0]), int(self.trajectory_points[i + 1][1]))
                    cv2.line(frame, pt1, pt2, (255, 0, 255), 2)

                for pt in self.trajectory_points:
                    cv2.circle(frame, (int(pt[0]), int(pt[1])), 3, (255, 255, 0), -1)

            # 矩形标定
            if self.paint_mode == "plane":
                if (self.rect_confirmed or self.rect_x0 >= 0) and self.rect_x1 >= 0:
                    x0 = np.clip(self.rect_x0, rx0, rx1)
                    y0 = np.clip(self.rect_y0, ry0, ry1)
                    x1 = np.clip(self.rect_x1, rx0, rx1)
                    y1 = np.clip(self.rect_y1, ry0, ry1)
                    cv2.rectangle(frame, (int(x0), int(y0)), (int(x1), int(y1)), (0, 255, 0), 2)
                    cv2.putText(frame, "Selected Area", (int(x0), int(y0) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # 显示深度信息
                    if self.rect_depth > 0:
                        cv2.putText(frame, f"Avg Depth: {self.rect_depth:.1f}mm",
                                    (int(x0), int(y1) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                self.rect_signal.emit(self.rect_x0, self.rect_y0, self.rect_x1, self.rect_y1)

            # 圆形标定
            elif self.paint_mode == "circle":
                if (self.circle_confirmed or self.cx >= 0) and self.radius > 0:
                    cv2.circle(frame, (int(self.cx), int(self.cy)), 3, (255, 0, 0), -1)
                    max_r = min(self.cx - rx0, rx1 - self.cx, self.cy - ry0, ry1 - self.cy)
                    r = min(self.radius, max_r)
                    cv2.circle(frame, (int(self.cx), int(self.cy)), int(r), (255, 165, 0), 2)
                    cv2.putText(frame, f"R={int(r)}", (int(self.cx) + int(r) + 5, int(self.cy)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 1)

                    # 显示圆心深度
                    if self.circle_depth > 0:
                        cv2.putText(frame, f"Center Depth: {self.circle_depth:.1f}mm",
                                    (int(self.cx) + 10, int(self.cy) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

                self.circle_signal.emit(self.cx, self.cy, self.radius)

            self.frame_signal.emit(frame)
            time.sleep(0.03)

        if self.rs_camera:
            self.rs_camera.stop()

    def reset_all(self):
        self.rect_x0, self.rect_y0 = -1, -1
        self.rect_x1, self.rect_y1 = -1, -1
        self.rect_confirmed = False
        self.rect_depth = 0
        self.cx, self.cy = -1, -1
        self.radius = 0
        self.circle_confirmed = False
        self.circle_depth = 0
        self.clear_trajectory()

    def depth_snapshot(self):
        with self.frame_lock:
            if self.latest_depth is None or time.monotonic() - self.latest_time > CONFIG['max_frame_age_s']:
                raise ValueError("缺少有效深度帧，请重新采集")
            return self.latest_depth



# ======================
# xArm 机械臂线程（支持 3D 坐标）
# ======================
class ArmThread(QThread):
    pose_signal = Signal(list)
    log_signal = Signal(str)
    status_signal = Signal(str)
    paint_finished = Signal()

    def __init__(self):
        super().__init__()
        self.arm = None
        self.connected = False
        self.ip = ""
        self.running = True
        self.is_painting = False

        self.x, self.y, self.z = CONFIG["initial_pose_mm_deg"][:3]
        self.roll, self.pitch, self.yaw = CONFIG["initial_pose_mm_deg"][3:]
        self.executor = TrajectoryExecutor()

    def img2arm(self, x, y):
        return map_2d(x, y, CONFIG['mapping_2d'])


    def connect_arm(self, ip):
        if settings.OFFLINE:
            self.log_signal.emit("离线模式不连接机械臂")
            return False
        if XArmAPI is None:
            self.log_signal.emit("缺少 xarm-python-sdk，请安装 requirements-hardware-v2.txt")
            return False
        if not CONFIG['mapping_reviewed']:
            self.log_signal.emit("请先核对 config/settings.json 中的实验坐标映射，见 docs/software.md")
            return False
        try:
            self.disconnect_arm()
            self.ip = ip
            self.arm = XArmAPI(ip, do_not_open=True)
            self.arm.connect()
            if not self.arm.connected:
                raise ConnectionError("机械臂未建立连接")
            for code in (self.arm.motion_enable(enable=True), self.arm.set_mode(0), self.arm.set_state(0)):
                if code != 0:
                    raise RuntimeError(f"机械臂初始化失败，返回码 {code}")
            self.connected = True
            self.log_signal.emit(f"xArm 已连接：{ip}")
            self.status_signal.emit("已连接")
            return True
        except Exception as exc:
            self.disconnect_arm()
            self.log_signal.emit(f"连接失败：{exc}")
            return False


    def disconnect_arm(self):
        self.executor.cancel()
        self.connected = False
        if self.arm:
            try:
                self.arm.disconnect()
            except Exception as exc:
                self.log_signal.emit(f"断开连接失败：{exc}")
        self.status_signal.emit("未连接")


    def move_to(self, tx, ty, tz=None, speed=50, wait=True):
        if not self.connected or not self.arm:
            self.log_signal.emit(" 未连接机械臂")
            return False
        if tz is None:
            tz = self.z
        pose = [tx, ty, tz, self.roll, self.pitch, self.yaw]
        try:
            self.log_signal.emit(f" 移动：X={tx:.1f} Y={ty:.1f} Z={tz:.1f}")
            code = self.arm.set_position(*pose, speed=speed, wait=wait)
            if code == 0:
                self.x, self.y, self.z = tx, ty, tz
                return True
            else:
                self.log_signal.emit(f" 移动返回码：{code}")
                return False
        except Exception as e:
            self.log_signal.emit(f" 移动失败：{str(e)}")
            return False

    def execute_trajectory_3d(self, points_3d, speed=30):
        self.is_painting = True
        try:
            return self.executor.execute(points_3d,
                lambda x, y, z: self.move_to(x, y, z, speed=speed, wait=True),
                lambda: self.connected and bool(self.arm and self.arm.connected))
        finally:
            self.is_painting = False


    def execute_trajectory(self, img_points, speed=30):
        try:
            points = [(*self.img2arm(x, y), self.z) for x, y in img_points]
        except ValueError as exc:
            self.log_signal.emit(str(exc))
            return Outcome.INVALID
        return self.execute_trajectory_3d(points, speed)


    def move_home(self):
        if not self.connected:
            return
        try:
            code = self.arm.move_gohome(speed=50, wait=True)
            if code != 0:
                raise RuntimeError(f"返回码 {code}")
            code, pos = self.arm.get_position()
            if code == 0:
                self.x, self.y, self.z = pos[:3]
        except Exception as exc:
            self.log_signal.emit(f"回原点失败：{exc}")


    def emergency_stop(self):
        self.executor.cancel()
        self.is_painting = False
        if self.arm and self.connected:
            try:
                code = self.arm.emergency_stop()
                self.log_signal.emit(f"已发送机械臂停止命令，返回值 {code}；喷枪请使用独立遥控关闭")
            except Exception as exc:
                self.log_signal.emit(f"停止命令失败：{exc}")
        else:
            self.log_signal.emit("机械臂未连接；喷枪使用独立遥控开关")


    def run(self):
        while self.running:
            if self.connected and self.arm:
                try:
                    code, pos = self.arm.get_position()
                    if code == 0 and len(pos) >= 3:
                        self.x, self.y, self.z = pos[0], pos[1], pos[2]
                except:
                    pass
            self.pose_signal.emit([self.x, self.y, self.z])
            time.sleep(0.2)

    def stop(self):
        self.running = False
        self.executor.cancel()
        self.disconnect_arm()



# ======================
# 轨迹预览对话框（支持 3D）
# ======================
class TrajectoryPreviewDialog(QDialog):
    def __init__(self, points, points_3d=None, parent=None):
        super().__init__(parent)
        self.setStyleSheet("QDialog { background: #1a1a1a; } QLabel { color: white; } QPushButton { color: white; background: #333; padding: 8px; }")
        self.points = points
        self.points_3d = points_3d
        self.setWindowTitle("喷涂轨迹预览 (3D)")
        self.setFixedSize(750, 620)
        self.setModal(True)
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(15)

        info_text = f"轨迹点数：{len(self.points)} 个"
        if self.points_3d:
            info_text += " | 3D 模式已启用"
        info = QLabel(info_text)
        info.setStyleSheet("color: white; font-size: 13px; font-weight: bold;")
        layout.addWidget(info)

        self.canvas = QLabel()
        self.canvas.setFixedSize(640, 480)
        self.canvas.setStyleSheet("background-color: #1a1a1a; border: 2px solid #555; border-radius: 8px;")
        self.canvas.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.canvas)

        self.draw_preview()

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(20)
        self.confirm_btn = QPushButton(" 确认执行")
        self.confirm_btn.setFixedSize(140, 40)
        self.confirm_btn.setObjectName("green")
        self.cancel_btn = QPushButton(" 取消")
        self.cancel_btn.setFixedSize(120, 40)

        btn_layout.addStretch()
        btn_layout.addWidget(self.confirm_btn)
        btn_layout.addWidget(self.cancel_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        self.confirm_btn.clicked.connect(self.accept)
        self.cancel_btn.clicked.connect(self.reject)

    def draw_preview(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)

        cv2.rectangle(img, (50, 50), (590, 430), (0, 255, 255), 2)
        cv2.putText(img, "Paint Area", (60, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if len(self.points) > 1:
            for i in range(len(self.points) - 1):
                pt1 = (int(self.points[i][0]), int(self.points[i][1]))
                pt2 = (int(self.points[i + 1][0]), int(self.points[i + 1][1]))
                cv2.line(img, pt1, pt2, (255, 0, 255), 2)

            start = (int(self.points[0][0]), int(self.points[0][1]))
            end = (int(self.points[-1][0]), int(self.points[-1][1]))
            cv2.circle(img, start, 6, (0, 255, 0), -1)
            cv2.circle(img, end, 6, (0, 0, 255), -1)

            for pt in self.points:
                cv2.circle(img, (int(pt[0]), int(pt[1])), 2, (255, 255, 0), -1)

        # 添加 3D 模式提示
        if self.points_3d:
            cv2.putText(img, "3D Trajectory Mode", (520, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.putText(img, "● Start", (520, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(img, "● End", (520, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(img, "─ Trajectory", (520, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        h, w, ch = img.shape
        qimg = QImage(img.data, w, h, ch * w, QImage.Format_BGR888)
        self.canvas.setPixmap(QPixmap.fromImage(qimg))


# ======================
# 主窗口
# ======================

class TrajectoryWorker(QThread):
    def __init__(self, arm, points_2d, points_3d, parent=None):
        super().__init__(parent)
        self.arm, self.points_2d, self.points_3d = arm, points_2d, points_3d
        self.result = Outcome.FAILED

    def run(self):
        try:
            if self.points_3d is not None:
                self.result = self.arm.execute_trajectory_3d(self.points_3d)
            else:
                self.result = self.arm.execute_trajectory(self.points_2d)
        except Exception as exc:
            self.arm.log_signal.emit(f"轨迹执行异常：{exc}")
            self.result = Outcome.FAILED


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("xArm 视觉喷涂系统 (RealSense 3D)")
        self.setFixedSize(1480, 850)
        self.setStyleSheet(self.get_style())

        self.cam = CameraThread()
        self.arm = ArmThread()
        self.paint_mode = "plane"
        self.current_trajectory = []
        self.current_trajectory_3d = []
        self.use_3d = not settings.OFFLINE
        self.worker = None

        self.init_ui()
        self.bind_events()
        if settings.OFFLINE:
            self.connect_btn.setEnabled(False)
            self.log.addItem("离线预览：相机、机械臂和喷枪均未连接")
        self.on_3d_mode_changed()
        self.cam.start()
        self.arm.start()

    def get_style(self):
        return """
        QMainWindow { background-color: #0a0a0a; }
        QGroupBox {
            background-color: #1a1a1a; color: white;
            border: 1px solid #444; border-radius: 8px; margin-top: 8px; font-weight: bold;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px 0 5px;
        }
        QLabel { color: white; font-size: 10pt; }
        QPushButton {
            background-color: #2a2a2a; color: white; border: none; border-radius: 5px;
            font-size: 11pt; font-weight: bold;
        }
        QPushButton:hover { background-color: #007acc; }
        QPushButton#green { background-color: #2E8B57; min-height: 30px; }
        QPushButton#green:hover { background-color: #3cb371; }
        QPushButton#red { background-color: #cc0000; min-height: 30px; }
        QPushButton#red:hover { background-color: #ff3333; }
        QPushButton#blue { background-color: #4682B4; min-height: 30px; }
        QPushButton#blue:hover { background-color: #5a9bcf; }
        QPushButton#orange { background-color: #e67e22; min-height: 30px; }
        QPushButton#orange:hover { background-color: #f39c12; }
        QLineEdit {
            background-color: #222; color: white; padding: 6px;
            border: 1px solid #555; border-radius: 5px; min-height: 20px;
        }
        QListWidget { background-color: #111; color: #ccc; border: 1px solid #444; }
        QRadioButton { color: white; font-size: 11pt; spacing: 8px; }
        QSpinBox {
            background-color: #222; color: white; border: 1px solid #555;
            border-radius: 5px; padding: 4px; min-width: 80px;
        }
        QFormLayout { spacing: 12px; }
        QCheckBox { color: white; font-size: 11pt; }
        """

    def init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(20, 20, 20, 20)
        main_layout.setSpacing(20)

        # ========== 左侧面板 ==========
        left_layout = QVBoxLayout()
        left_layout.setSpacing(12)

        title = QLabel("xArm 视觉喷涂控制系统 (RealSense 3D)")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 16px; color: #00ccff; font-weight: bold; padding: 5px;")
        left_layout.addWidget(title)

        self.status_label = QLabel("状态：已启动 | 机械臂：未连接 | 3D模式：启用")
        self.status_label.setStyleSheet("color: #00ff00; font-size: 12px; padding: 5px;")
        left_layout.addWidget(self.status_label)

        # 连接组
        conn_group = QGroupBox(" xArm 连接")
        conn_layout = QHBoxLayout(conn_group)
        conn_layout.setSpacing(10)
        self.ip_edit = QLineEdit(CONFIG["arm_ip"])
        self.ip_edit.setFixedWidth(130)
        self.connect_btn = QPushButton("连接")
        self.connect_btn.setFixedSize(80, 32)
        self.connect_btn.setObjectName("blue")
        self.disconnect_btn = QPushButton("断开")
        self.disconnect_btn.setFixedSize(80, 32)
        self.disconnect_btn.setObjectName("red")
        conn_layout.addWidget(QLabel("xArm IP："))
        conn_layout.addWidget(self.ip_edit)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.disconnect_btn)
        conn_layout.addStretch()
        left_layout.addWidget(conn_group)

        # 喷涂模式组
        mode_group = QGroupBox(" 喷涂模式")
        mode_layout = QHBoxLayout(mode_group)
        mode_layout.setSpacing(20)
        self.plane_mode = QRadioButton("平面矩形")
        self.circle_mode = QRadioButton("圆形")
        self.plane_mode.setChecked(True)
        mode_layout.addWidget(self.plane_mode)
        mode_layout.addWidget(self.circle_mode)
        mode_layout.addStretch()
        left_layout.addWidget(mode_group)

        # 3D 模式选项
        self.use_3d_checkbox = QCheckBox("启用 3D 深度模式 (RealSense)")
        self.use_3d_checkbox.setChecked(self.use_3d)
        self.use_3d_checkbox.setStyleSheet("color: #00ccff;")
        left_layout.addWidget(self.use_3d_checkbox)

        # 轨迹参数组
        param_group = QGroupBox(" 轨迹参数")
        param_layout = QFormLayout(param_group)
        param_layout.setSpacing(10)
        self.step_spin = QSpinBox()
        self.step_spin.setRange(5, 50)
        self.step_spin.setValue(20)
        self.step_spin.setSuffix(" px")
        self.step_spin.setFixedWidth(100)
        step_layout = QHBoxLayout()
        step_layout.addWidget(self.step_spin)
        step_layout.addStretch()
        param_layout.addRow("轨迹间距：", step_layout)
        left_layout.addWidget(param_group)

        # 坐标显示组
        data_group = QGroupBox(" 图像 → 机械臂坐标")
        data_layout = QFormLayout(data_group)
        data_layout.setSpacing(8)
        self.p1 = QLabel("(0,0) → (0,0,0)")
        self.p2 = QLabel("(0,0) → (0,0,0)")
        self.p1.setStyleSheet("font-family: monospace; font-size: 11pt;")
        self.p2.setStyleSheet("font-family: monospace; font-size: 11pt;")
        data_layout.addRow("点1：", self.p1)
        data_layout.addRow("点2：", self.p2)
        left_layout.addWidget(data_group)

        # 手动控制组
        ctrl_group = QGroupBox(" 手动控制")
        grid = QGridLayout(ctrl_group)
        grid.setSpacing(12)

        btn_size = (70, 45)

        self.xp = QPushButton("X +")
        self.xm = QPushButton("X -")
        self.yp = QPushButton("Y +")
        self.ym = QPushButton("Y -")
        self.zp = QPushButton("Z +")
        self.zm = QPushButton("Z -")
        self.home = QPushButton(" 回原点")

        for btn in [self.xp, self.xm, self.yp, self.ym, self.zp, self.zm]:
            btn.setFixedSize(*btn_size)
            btn.setStyleSheet("font-size: 12pt; font-weight: bold;")

        self.home.setFixedSize(150, 45)
        self.home.setStyleSheet("font-size: 12pt; font-weight: bold; background-color: #555;")

        grid.addWidget(self.xp, 0, 0)
        grid.addWidget(self.xm, 0, 1)
        grid.addWidget(self.yp, 1, 0)
        grid.addWidget(self.ym, 1, 1)
        grid.addWidget(self.zp, 2, 0)
        grid.addWidget(self.zm, 2, 1)
        grid.addWidget(self.home, 3, 0, 1, 2)

        left_layout.addWidget(ctrl_group)

        # 系统功能组
        func_group = QGroupBox(" 系统功能")
        func_layout = QHBoxLayout(func_group)
        func_layout.setSpacing(15)

        self.clear_btn = QPushButton(" 清除区域")
        self.preview_btn = QPushButton(" 预览轨迹")
        self.start = QPushButton("▶ 执行轨迹")
        self.stop = QPushButton(" 紧急停止")

        for btn in [self.clear_btn, self.preview_btn, self.start, self.stop]:
            btn.setFixedSize(130, 40)

        self.clear_btn.setObjectName("blue")
        self.preview_btn.setObjectName("orange")
        self.start.setObjectName("green")
        self.stop.setObjectName("red")

        func_layout.addWidget(self.clear_btn)
        func_layout.addWidget(self.preview_btn)
        func_layout.addWidget(self.start)
        func_layout.addWidget(self.stop)
        left_layout.addWidget(func_group)

        # 日志组
        log_group = QGroupBox(" 运行日志")
        log_layout = QVBoxLayout(log_group)
        self.log = QListWidget()
        self.log.setMaximumHeight(160)
        self.log.setStyleSheet("font-size: 10pt;")
        log_layout.addWidget(self.log)
        left_layout.addWidget(log_group)

        # ========== 右侧面板 ==========
        right_layout = QVBoxLayout()
        cam_group = QGroupBox(" 视觉监控 (RealSense 3D)")
        cam_layout = QVBoxLayout(cam_group)
        self.video_label = QLabel()
        self.video_label.setFixedSize(640, 480)
        self.video_label.setMouseTracking(True)
        self.video_label.mousePressEvent = self.mousePressEvent
        self.video_label.mouseMoveEvent = self.mouseMoveEvent
        self.video_label.setStyleSheet("background-color: #000; border: 2px solid #555; border-radius: 10px;")
        cam_layout.addWidget(self.video_label, alignment=Qt.AlignCenter)
        self.area_label = QLabel("平面投影面积估计：--；喷枪使用独立遥控开关")
        self.area_label.setWordWrap(True)
        right_layout.addWidget(self.area_label)
        right_layout.addWidget(cam_group)

        main_layout.addLayout(left_layout, 35)
        main_layout.addLayout(right_layout, 65)

    def bind_events(self):
        self.cam.frame_signal.connect(self.show_frame)
        self.cam.rect_signal.connect(self.show_rect)
        self.cam.circle_signal.connect(self.show_circle)
        self.arm.log_signal.connect(self.log.addItem)
        self.arm.status_signal.connect(self.update_status)

        self.plane_mode.clicked.connect(lambda: self.set_mode("plane"))
        self.circle_mode.clicked.connect(lambda: self.set_mode("circle"))
        self.clear_btn.clicked.connect(self.clear_area)
        self.preview_btn.clicked.connect(self.preview_trajectory)
        self.start.clicked.connect(self.preview_trajectory)
        self.stop.clicked.connect(self.arm.emergency_stop)

        self.connect_btn.clicked.connect(lambda: self.arm.connect_arm(self.ip_edit.text().strip()))
        self.disconnect_btn.clicked.connect(self.arm.disconnect_arm)

        self.xp.clicked.connect(lambda: self.move_axis("x", 5))
        self.xm.clicked.connect(lambda: self.move_axis("x", -5))
        self.yp.clicked.connect(lambda: self.move_axis("y", 5))
        self.ym.clicked.connect(lambda: self.move_axis("y", -5))
        self.zp.clicked.connect(lambda: self.move_axis("z", 5))
        self.zm.clicked.connect(lambda: self.move_axis("z", -5))
        self.home.clicked.connect(self.arm.move_home)

        self.use_3d_checkbox.stateChanged.connect(self.on_3d_mode_changed)

    def on_3d_mode_changed(self):
        self.use_3d = self.use_3d_checkbox.isChecked()
        self.cam.rect_depth = self.cam.circle_depth = 0
        self.current_trajectory = []
        self.current_trajectory_3d = []
        self.cam.clear_trajectory()
        status = "启用" if self.use_3d else "禁用"
        self.log.addItem(f"[3D模式] {status}")
        self.status_label.setText(
            f"状态：运行中 | 机械臂：{'已连接' if self.arm.connected else '未连接'} | 3D模式：{status}")

    def update_status(self, s):
        mode_status = "启用" if self.use_3d else "禁用"
        self.status_label.setText(f"状态：运行中 | 机械臂：{s} | 3D模式：{mode_status}")

    def move_axis(self, axis, step):
        if not self.arm.connected:
            QMessageBox.warning(self, "提示", "未连接 xArm")
            return
        tx, ty, tz = self.arm.x, self.arm.y, self.arm.z
        if axis == "x":
            tx += step
        elif axis == "y":
            ty += step
        elif axis == "z":
            tz += step
        self.arm.move_to(tx, ty, tz, speed=30)

    def set_mode(self, mode):
        self.paint_mode = mode
        self.cam.paint_mode = mode
        self.clear_area()

    def clear_area(self):
        self.cam.reset_all()
        self.area_label.setText("平面投影面积估计：--；喷枪使用独立遥控开关")
        self.current_trajectory = []
        self.current_trajectory_3d = []
        self.cam.show_trajectory = False
        self.p1.setText("(0,0) → (0,0,0)")
        self.p2.setText("(0,0) → (0,0,0)")
        self.log.addItem("[清除] 已清空所有喷涂区域和轨迹")

    def generate_trajectory(self):
        try:
            step = self.step_spin.value()
            if self.paint_mode == 'plane':
                if not self.cam.rect_confirmed:
                    raise ValueError("请在画面中依次点击矩形的两个对角点")
                points = rectangle_path(self.cam.rect_x0, self.cam.rect_y0, self.cam.rect_x1, self.cam.rect_y1, step)
            else:
                if not self.cam.circle_confirmed:
                    raise ValueError("请依次点击圆心和圆周点")
                points = circle_path(self.cam.cx, self.cam.cy, self.cam.radius, step)
            bounds = CONFIG['paint_bounds_px']
            if any(not bounds[0] - 1e-6 <= x <= bounds[2] + 1e-6 or not bounds[1] - 1e-6 <= y <= bounds[3] + 1e-6 for x, y in points):
                raise ValueError("选区超出黄色作业框，请重新选择")
            points_3d = None
            if self.use_3d:
                frame = self.cam.depth_snapshot()
                if not self.cam.rs_camera:
                    raise ValueError("未连接深度相机")
                points_3d, depth = project_path(points,
                    lambda x, y: self.cam.rs_camera.get_depth_at_pixel(x, y, frame),
                    self.cam.rs_camera.camera_info, CONFIG['mapping_3d'], CONFIG['max_depth_mm'])
                self.cam.rect_depth = self.cam.circle_depth = depth
                intr = self.cam.rs_camera.camera_info
                if self.paint_mode == 'plane':
                    pixel_area = abs((self.cam.rect_x1-self.cam.rect_x0)*(self.cam.rect_y1-self.cam.rect_y0))
                else:
                    pixel_area = math.pi * self.cam.radius ** 2
                area = pixel_area * depth ** 2 / (intr['fx'] * intr['fy']) / 1e6
                self.area_label.setText(f"平面投影面积估计：{area:.4f} m²；喷枪使用独立遥控开关")
            else:
                self.cam.rect_depth = self.cam.circle_depth = 0
                self.area_label.setText("二维轨迹预览；实际面积需有效深度，喷枪使用独立遥控开关")
            self.log.addItem(f"已生成 {len(points)} 个轨迹点；{'三维' if self.use_3d else '二维'}模式")
            return points, points_3d
        except (ValueError, KeyError, TypeError) as exc:
            self.cam.rect_depth = self.cam.circle_depth = 0
            self.current_trajectory = []
            self.current_trajectory_3d = []
            self.cam.clear_trajectory()
            self.area_label.setText("轨迹生成失败，面积估计已清除")
            self.log.addItem(f"轨迹生成失败：{exc}")
            return None, None


    def preview_trajectory(self):
        points_2d, points_3d = self.generate_trajectory()
        if points_2d is None:
            return

        self.current_trajectory = points_2d
        self.current_trajectory_3d = points_3d
        self.cam.set_trajectory_and_show(points_2d)

        dialog = TrajectoryPreviewDialog(points_2d, points_3d, self)
        result = dialog.exec()

        if result == QDialog.Accepted:
            self.log.addItem("[预览] 用户确认执行轨迹")
            self.execute_paint_trajectory()
        else:
            self.log.addItem("[预览] 用户取消喷涂")
            self.cam.show_trajectory = False

    def execute_paint_trajectory(self):
        if settings.OFFLINE or not self.arm.connected:
            self.log.addItem("轨迹预览完成；机械臂未连接，未执行运动")
            return
        if self.worker is not None:
            self.log.addItem("任务尚未结束")
            return
        if not self.current_trajectory:
            self.log.addItem("没有可执行的轨迹")
            return
        if self.use_3d and not self.current_trajectory_3d:
            self.log.addItem("缺少三维轨迹，请重新采集深度")
            return
        self.arm.executor.reset()
        self.worker = TrajectoryWorker(self.arm, list(self.current_trajectory),
            list(self.current_trajectory_3d) if self.current_trajectory_3d else None, self)
        self._set_task_controls(False)
        self.worker.finished.connect(self.on_paint_finished)
        self.log.addItem("开始执行机械臂轨迹；喷枪使用独立遥控开关")
        self.worker.start()


    def start_paint(self):
        self.preview_trajectory()


    def on_paint_finished(self):
        worker = self.worker
        if worker is None:
            return
        result = worker.result
        text = {Outcome.COMPLETE: '机械臂轨迹执行完成', Outcome.STOPPED: '任务已中断',
                Outcome.FAILED: '运动失败，任务停止', Outcome.DISCONNECTED: '连接已断开，任务停止',
                Outcome.INVALID: '轨迹无效，未执行运动'}[result]
        self.log.addItem(text)
        self._set_task_controls(True)
        self.worker = None
        worker.deleteLater()


    def show_rect(self, x0, y0, x1, y1):
        if x0 >= 0 and x1 >= 0:
            self.p1.setText(f"图像点 ({x0:.0f}, {y0:.0f})")
            self.p2.setText(f"图像点 ({x1:.0f}, {y1:.0f})；平均深度 {self.cam.rect_depth:.1f} mm")


    def show_circle(self, cx, cy, r):
        if cx >= 0 and r > 0:
            self.p1.setText(f"圆心 ({cx:.0f}, {cy:.0f})")
            self.p2.setText(f"半径 {r:.0f} px")


    def mousePressEvent(self, evt):
        if evt.button() != Qt.LeftButton or self.worker is not None:
            return
        pos = self.video_label.mapFromGlobal(evt.globalPosition().toPoint())
        x, y = pos.x(), pos.y()
        x0, y0, x1, y1 = CONFIG['paint_bounds_px']
        if not (x0 <= x <= x1 and y0 <= y <= y1):
            return
        self.current_trajectory = []
        self.current_trajectory_3d = []
        self.cam.clear_trajectory()
        self.area_label.setText("选区已更新；等待轨迹生成")
        if self.paint_mode == 'plane':
            if self.cam.rect_confirmed or self.cam.rect_x0 < 0:
                self.cam.rect_confirmed = False
                self.cam.rect_x0, self.cam.rect_y0 = x, y
                self.cam.rect_x1 = self.cam.rect_y1 = -1
            else:
                self.cam.rect_x1, self.cam.rect_y1 = x, y
                self.cam.rect_confirmed = True
        else:
            if self.cam.circle_confirmed or self.cam.cx < 0:
                self.cam.cx, self.cam.cy = x, y
                self.cam.radius = 0
                self.cam.circle_confirmed = False
            else:
                radius = math.hypot(x-self.cam.cx, y-self.cam.cy)
                self.cam.radius = min(radius, self.cam.cx-x0, x1-self.cam.cx, self.cam.cy-y0, y1-self.cam.cy)
                self.cam.circle_confirmed = self.cam.radius > 0


    def mouseMoveEvent(self, evt):
        if self.worker is not None:
            return
        pos = self.video_label.mapFromGlobal(evt.globalPosition().toPoint())
        x0, y0, x1, y1 = CONFIG['paint_bounds_px']
        x, y = min(x1, max(x0, pos.x())), min(y1, max(y0, pos.y()))
        if self.paint_mode == 'plane' and self.cam.rect_x0 >= 0 and not self.cam.rect_confirmed:
            self.cam.rect_x1, self.cam.rect_y1 = x, y
        elif self.paint_mode == 'circle' and self.cam.cx >= 0 and not self.cam.circle_confirmed:
            self.cam.radius = min(math.hypot(x-self.cam.cx, y-self.cam.cy),
                self.cam.cx-x0, x1-self.cam.cx, self.cam.cy-y0, y1-self.cam.cy)


    @Slot(np.ndarray)
    def show_frame(self, img):
        h, w, ch = img.shape
        qimg = QImage(img.data, w, h, ch * w, QImage.Format_BGR888)
        self.video_label.setPixmap(QPixmap.fromImage(qimg))

    def closeEvent(self, evt):
        self.cam.running = False
        if self.worker is not None:
            self.arm.emergency_stop()
            if not self.worker.wait(3000):
                self.log.addItem("等待当前运动返回，请稍后再次关闭窗口")
                evt.ignore()
                return
        self.arm.stop()
        self.cam.wait()
        self.arm.wait()
        evt.accept()


    def _set_task_controls(self, enabled):
        for button in (self.start, self.preview_btn, self.clear_btn, self.connect_btn,
                       self.disconnect_btn, self.plane_mode, self.circle_mode,
                       self.use_3d_checkbox, self.step_spin, self.xp, self.xm, self.yp,
                       self.ym, self.zp, self.zm, self.home):
            button.setEnabled(enabled)
        if settings.OFFLINE:
            self.connect_btn.setEnabled(False)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
