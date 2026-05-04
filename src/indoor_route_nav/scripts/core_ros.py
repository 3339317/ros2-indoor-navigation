#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import math
import time
import json
import yaml
import threading

import numpy as np
import open3d as o3d

try:
    import cv2
except Exception:
    cv2 = None

try:
    from cv_bridge import CvBridge
except Exception:
    CvBridge = None

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Image


# =========================================================
# 核心状态与逻辑
# =========================================================
class AppCore:
    def __init__(self):
        self.lock = threading.RLock()

        # =================================================
        # 控制参数
        # =================================================
        self.max_vx = 0.70
        self.max_wz = 0.50
        self.min_vx = 0.12
        self.min_wz = 0.10
        self.arrival_dist = 0.18
        self.arrival_angle = 5.0
        self.alpha = 0.18
        self.obstacle_threshold = 1.5
        self.resume_delay = 0.5
        self.deceleration_alpha = 0.10
        self.use_obstacle_avoidance = True  # 是否启用避障
        self.obstacle_distance = 999.0
        self.obstacle_boxes = []
        self.obstacle_clusters = []

        self.lookahead_distance = 1.00
        self.path_yaw_kp = 0.020
        self.final_yaw_kp = 0.020
        self.end_slowdown_distance = 1.50
        self.heading_slow_angle_deg = 35.0

        # =================================================
        # 地图 / 路线
        # =================================================
        self.sample_step = 3
        self.route_sample_gap = 0.35

        self.pcd_path = ""
        self.points_xyz = []
        self.points_rgb = []
        self.points_xy = []  # 兼容旧接口

        self.route_polyline = []
        self.route_cumlen = []
        self.drivable_areas = []
        self.connectors = []

        # revision，用于前端判断是否需要拉取完整状态
        self.map_revision = 0
        self.route_revision = 0
        self.area_revision = 0

        # =================================================
        # UI / 状态
        # =================================================
        self.mode = "none"
        self.status_text = "等待操作"

        # =================================================
        # ROS 状态
        # =================================================
        self.current_pose = None
        self.localized = False

        self.current_vx = 0.0
        self.current_wz = 0.0
        self.is_obstacle_paused = False

        # =================================================
        # 导航状态
        # =================================================
        self.is_auto_moving = False
        self.stage = "idle"   # idle / path_tracking / final_align
        self.nav_progress_idx = 0
        self.lookahead_target = None
        self.final_target_yaw = 0.0
        self.use_cpp_controller = True

        self.external_goal_request = None
        self.external_goal_seq = 0

        self.original_goal = None
        self.obstacle_paused_at = 0.0
        self.last_replan_at = 0.0
        self.replan_cooldown_sec = 3.0
        self.replan_wait_sec = 1.5
        self.replan_max_count = 5
        self.replan_count = 0
        self._replan_flag = False

        # =================================================
        # 图像
        # =================================================
        self.camera_topic = "/camera/image_raw"
        self.camera_frame_jpeg = None
        self.camera_frame_time = 0.0

        # =================================================
        # 外部话题
        # =================================================
        self.nav_start_topic = "/nav/start"
        self.nav_stop_topic = "/nav/stop"
        self.nav_done_topic = "/nav/done"
        self.external_goal_topic = "/indoor_route_nav/goal"

        self.initialpose_frame = "map"

    # -----------------------------------------------------
    # revision
    # -----------------------------------------------------
    def _bump_map_rev(self):
        self.map_revision += 1

    def _bump_route_rev(self):
        self.route_revision += 1

    def _bump_area_rev(self):
        self.area_revision += 1

    # -----------------------------------------------------
    # 基础工具
    # -----------------------------------------------------
    def set_status(self, text: str):
        with self.lock:
            self.status_text = str(text)

    def normalize_angle_deg(self, ang):
        return (ang + 180.0) % 360.0 - 180.0

    def dist2d(self, ax, ay, bx, by):
        return math.hypot(ax - bx, ay - by)

    def dist3d(self, ax, ay, az, bx, by, bz):
        return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)

    def point_dist3d(self, p, q):
        return self.dist3d(
            float(p["x"]), float(p["y"]), float(p.get("z", 0.0)),
            float(q["x"]), float(q["y"]), float(q.get("z", 0.0))
        )

    def _normalize_path_point(self, p):
        if isinstance(p, dict):
            return {
                "x": float(p["x"]),
                "y": float(p["y"]),
                "z": float(p.get("z", 0.0)),
            }
        if isinstance(p, (list, tuple, np.ndarray)):
            arr = np.asarray(p).reshape(-1)
            if arr.size >= 2:
                return {
                    "x": float(arr[0]),
                    "y": float(arr[1]),
                    "z": float(arr[2]) if arr.size >= 3 else 0.0,
                }
        raise RuntimeError(f"无效路线点: {p}")

    def _reset_navigation_state_locked(self):
        self.is_auto_moving = False
        self.is_obstacle_paused = False
        self.stage = "idle"
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.lookahead_target = None
        self.nav_progress_idx = 0

    def _handle_navigation_completed_locked(self):
        self._reset_navigation_state_locked()
        self.route_polyline = []
        self._bump_route_rev()
        self.original_goal = None
        self.status_text = "路线导航完成"

    # -----------------------------------------------------
    # 图像
    # -----------------------------------------------------
    def update_camera_frame(self, jpeg_bytes: bytes):
        with self.lock:
            self.camera_frame_jpeg = jpeg_bytes
            self.camera_frame_time = time.time()

    def get_camera_frame(self):
        with self.lock:
            return self.camera_frame_jpeg, self.camera_frame_time

    # -----------------------------------------------------
    # 参数导出/导入
    # -----------------------------------------------------
    def get_control_params(self):
        return {
            "max_linear_x": self.max_vx,
            "max_angular_z": self.max_wz,
            "min_linear_x": self.min_vx,
            "min_angular_z": self.min_wz,
            "arrival_distance": self.arrival_dist,
            "arrival_angle_deg": self.arrival_angle,
            "alpha": self.alpha,
            "obstacle_threshold": self.obstacle_threshold,
            "resume_delay": self.resume_delay,
            "deceleration_alpha": self.deceleration_alpha,
            "use_obstacle_avoidance": self.use_obstacle_avoidance,
            "lookahead_distance": self.lookahead_distance,
            "path_yaw_kp": self.path_yaw_kp,
            "final_yaw_kp": self.final_yaw_kp,
            "end_slowdown_distance": self.end_slowdown_distance,
            "heading_slow_angle_deg": self.heading_slow_angle_deg,
        }

    def set_control_params(self, d):
        with self.lock:
            self.max_vx = max(0.01, float(d.get("max_linear_x", self.max_vx)))
            self.max_wz = max(0.01, float(d.get("max_angular_z", self.max_wz)))
            self.min_vx = max(0.0, float(d.get("min_linear_x", self.min_vx)))
            self.min_wz = max(0.0, float(d.get("min_angular_z", self.min_wz)))
            self.arrival_dist = max(0.01, float(d.get("arrival_distance", self.arrival_dist)))
            self.arrival_angle = max(0.1, float(d.get("arrival_angle_deg", self.arrival_angle)))
            self.alpha = min(1.0, max(0.001, float(d.get("alpha", self.alpha))))
            self.obstacle_threshold = max(0.01, float(d.get("obstacle_threshold", self.obstacle_threshold)))
            self.resume_delay = max(0.0, float(d.get("resume_delay", self.resume_delay)))
            self.deceleration_alpha = min(1.0, max(0.001, float(d.get("deceleration_alpha", self.deceleration_alpha))))
            self.use_obstacle_avoidance = bool(d.get("use_obstacle_avoidance", self.use_obstacle_avoidance))
            self.lookahead_distance = max(0.05, float(d.get("lookahead_distance", self.lookahead_distance)))
            self.path_yaw_kp = max(0.0001, float(d.get("path_yaw_kp", self.path_yaw_kp)))
            self.final_yaw_kp = max(0.0001, float(d.get("final_yaw_kp", self.final_yaw_kp)))
            self.end_slowdown_distance = max(0.05, float(d.get("end_slowdown_distance", self.end_slowdown_distance)))
            self.heading_slow_angle_deg = max(1.0, float(d.get("heading_slow_angle_deg", self.heading_slow_angle_deg)))

            if self.min_vx > self.max_vx:
                self.min_vx = self.max_vx
            if self.min_wz > self.max_wz:
                self.min_wz = self.max_wz

    def get_smoothing_params(self):
        return {
            "route_sample_gap": self.route_sample_gap,
        }

    def set_smoothing_params(self, d):
        with self.lock:
            self.route_sample_gap = max(0.02, float(d.get("route_sample_gap", self.route_sample_gap)))

    # -----------------------------------------------------
    # 地图
    # -----------------------------------------------------
    def load_pcd_file(self, path):
        with self.lock:
            pcd = o3d.io.read_point_cloud(path)
            pts = np.asarray(pcd.points)
            if len(pts) == 0:
                raise RuntimeError("点云为空")

            self.pcd_path = path

            sampled = pts[::self.sample_step].astype(float)
            self.points_xyz = sampled.tolist()
            self.points_xy = sampled[:, :2].tolist()

            if pcd.has_colors():
                cols = np.asarray(pcd.colors)
                if len(cols) == len(pts):
                    self.points_rgb = cols[::self.sample_step].astype(float).tolist()
                else:
                    self.points_rgb = []
            else:
                self.points_rgb = []

            self._bump_map_rev()
            self.set_status(f"已加载3D地图: {os.path.basename(path)}")

    # -----------------------------------------------------
    # 路线基础
    # -----------------------------------------------------
    def resample_polyline(self, pts, gap=0.35):
        pts = [self._normalize_path_point(p) for p in pts]
        if len(pts) < 2:
            return pts

        pts_arr = np.array(
            [[p["x"], p["y"], p.get("z", 0.0)] for p in pts],
            dtype=float
        )

        lengths = [0.0]
        for i in range(1, len(pts_arr)):
            lengths.append(lengths[-1] + np.linalg.norm(pts_arr[i] - pts_arr[i - 1]))

        total_len = lengths[-1]
        if total_len < 1e-6:
            return pts

        sample_s = np.arange(0.0, total_len, gap)
        if len(sample_s) == 0 or abs(sample_s[-1] - total_len) > 1e-9:
            sample_s = np.append(sample_s, total_len)

        result = []
        j = 0
        for s in sample_s:
            while j < len(lengths) - 2 and lengths[j + 1] < s:
                j += 1

            seg_len = lengths[j + 1] - lengths[j]
            if seg_len < 1e-6:
                p = pts_arr[j]
            else:
                ratio = (s - lengths[j]) / seg_len
                p = pts_arr[j] + ratio * (pts_arr[j + 1] - pts_arr[j])

            result.append({
                "x": float(p[0]),
                "y": float(p[1]),
                "z": float(p[2]),
            })

        return result

    def build_route_cumlen(self, poly):
        if len(poly) == 0:
            return []
        out = [0.0]
        for i in range(1, len(poly)):
            out.append(out[-1] + self.point_dist3d(poly[i], poly[i - 1]))
        return out

    def set_route_points(self, pts):
        with self.lock:
            route_points = [self._normalize_path_point(p) for p in pts]
            if len(route_points) < 2:
                raise RuntimeError("路线点数不足")

            route_points = self.resample_polyline(route_points, self.route_sample_gap)

            self._reset_navigation_state_locked()
            self.route_polyline = route_points
            self.route_cumlen = self.build_route_cumlen(self.route_polyline)
            if len(self.route_polyline) >= 2:
                b = self.route_polyline[-1]
                a = self.route_polyline[-2]
                self.final_target_yaw = math.degrees(math.atan2(b["y"] - a["y"], b["x"] - a["x"]))
            else:
                self.final_target_yaw = 0.0

            self._bump_route_rev()
            self.set_status(f"已设置路线，共 {len(self.route_polyline)} 点，等待开启导航")

    def set_external_route(self, pts, auto_start=False):
        # 为兼容旧接口保留 auto_start 参数，但这里不再自动启动导航
        self.set_route_points(pts)

    def set_external_route_bundle(self, data):
        if not isinstance(data, dict):
            raise RuntimeError("路线包必须是 JSON 对象")

        route = data.get("route", {})
        if isinstance(route, dict):
            points = route.get("polyline", [])
        else:
            points = route

        if not points:
            points = data.get("points") or data.get("route_points") or data.get("route_polyline") or []

        speed_params = data.get("speed_params") or data.get("control_params") or {}
        auto_start = bool(data.get("auto_start", False))

        if speed_params:
            self.set_control_params(speed_params)

        self.set_route_points(points)

        if auto_start:
            try:
                self.start_route_navigation()
                self.set_status(
                    f"已接收外部路线包并启动导航，路线点 {len(self.route_polyline)}"
                )
            except Exception as e:
                self.set_status(f"已接收外部路线包，自动启动失败: {e}")
        else:
            self.set_status(
                f"已接收外部路线包，路线点 {len(self.route_polyline)}，等待开启导航"
            )

    def clear_navigation_data(self):
        with self.lock:
            self._reset_navigation_state_locked()
            self.route_polyline = []
            self.route_cumlen = []

            self.set_status("已清除路线和导航状态")

    # 兼容旧接口
    def clear_route(self):
        self.clear_navigation_data()

    # -----------------------------------------------------
    # 可行驶区域
    # -----------------------------------------------------
    def _normalize_drivable_area(self, area, idx=0):
        if not isinstance(area, dict):
            raise RuntimeError("可行驶区域格式错误")

        raw_points = area.get("points", [])
        points = [self._normalize_path_point(p) for p in raw_points]
        if len(points) < 3:
            raise RuntimeError("可行驶区域至少需要3个点")

        z_vals = [float(p.get("z", 0.0)) for p in points]
        z_min = float(area.get("z_min", min(z_vals)))
        z_max = float(area.get("z_max", max(z_vals)))
        if z_min > z_max:
            z_min, z_max = z_max, z_min

        normalized = {
            "type": str(area.get("type", "polygon")),
            "name": str(area.get("name", f"Area{idx}")),
            "z_min": z_min,
            "z_max": z_max,
            "points": points,
        }
        if isinstance(area.get("centerline"), list):
            normalized["centerline"] = [
                self._normalize_path_point(p)
                for p in area.get("centerline", [])
            ]
        if "width" in area:
            normalized["width"] = float(area.get("width", 0.0))
        return normalized

    def add_drivable_area(self, area):
        with self.lock:
            narea = self._normalize_drivable_area(area, len(self.drivable_areas))
            self.drivable_areas.append(narea)
            self._bump_area_rev()
            self.set_status(f"已添加可行驶区域: {narea['name']}")

    def set_drivable_areas(self, areas):
        with self.lock:
            if not isinstance(areas, list):
                raise RuntimeError("可行驶区域列表格式错误")
            self.drivable_areas = [
                self._normalize_drivable_area(area, i)
                for i, area in enumerate(areas)
            ]
            self._bump_area_rev()
            self.set_status(f"已设置可行驶区域，共 {len(self.drivable_areas)} 个")

    def delete_drivable_area(self, idx):
        with self.lock:
            if idx < 0 or idx >= len(self.drivable_areas):
                raise RuntimeError("可行驶区域索引超出范围")
            removed = self.drivable_areas.pop(idx)
            self._bump_area_rev()
            self.set_status(f"已删除可行驶区域: {removed['name']}")

    def clear_drivable_areas(self):
        with self.lock:
            self.drivable_areas = []
            self._bump_area_rev()
            self.set_status("已清空可行驶区域")

    # -----------------------------------------------------
    # 楼层连接区 / 楼梯
    # -----------------------------------------------------
    def _normalize_connector(self, connector, idx=0):
        if not isinstance(connector, dict):
            raise RuntimeError("连接区格式错误")

        raw_points = connector.get("points", [])
        points = [self._normalize_path_point(p) for p in raw_points]
        if len(points) < 2:
            raise RuntimeError("连接区至少需要2个点")

        z_vals = [float(p.get("z", 0.0)) for p in points]
        z_min = float(connector.get("z_min", min(z_vals)))
        z_max = float(connector.get("z_max", max(z_vals)))
        if z_min > z_max:
            z_min, z_max = z_max, z_min

        ctype = str(connector.get("type", "stair"))
        return {
            "type": ctype,
            "name": str(connector.get("name", f"{ctype}_{idx}")),
            "from_floor": str(connector.get("from_floor", "")),
            "to_floor": str(connector.get("to_floor", "")),
            "z_min": z_min,
            "z_max": z_max,
            "points": points,
        }

    def add_connector(self, connector):
        with self.lock:
            nconn = self._normalize_connector(connector, len(self.connectors))
            self.connectors.append(nconn)
            self._bump_area_rev()
            self.set_status(f"已添加连接区: {nconn['name']}")

    def delete_connector(self, idx):
        with self.lock:
            if idx < 0 or idx >= len(self.connectors):
                raise RuntimeError("连接区索引超出范围")
            removed = self.connectors.pop(idx)
            self._bump_area_rev()
            self.set_status(f"已删除连接区: {removed['name']}")

    def clear_connectors(self):
        with self.lock:
            self.connectors = []
            self._bump_area_rev()
            self.set_status("已清空楼层连接区")

    def dump_system_params_yaml(self):
        with self.lock:
            data = {
                "control_params": self.get_control_params(),
                "smoothing_params": self.get_smoothing_params(),
                "map_display": {
                    "sample_step": self.sample_step,
                },
                "topics": {
                    "camera_topic": self.camera_topic,
                    "nav_start_topic": self.nav_start_topic,
                    "nav_stop_topic": self.nav_stop_topic,
                    "nav_done_topic": self.nav_done_topic,
                    "external_goal_topic": self.external_goal_topic,
                    "initialpose_frame": self.initialpose_frame,
                }
            }
            return yaml.dump(data, allow_unicode=True, sort_keys=False)

    def load_system_params_data(self, data):
        with self.lock:
            self.set_control_params(data.get("control_params", {}))
            self.set_smoothing_params(data.get("smoothing_params", {}))

            map_display = data.get("map_display", {})
            self.sample_step = max(1, int(map_display.get("sample_step", self.sample_step)))

            topics = data.get("topics", {})
            self.camera_topic = topics.get("camera_topic", self.camera_topic)
            self.nav_start_topic = topics.get("nav_start_topic", self.nav_start_topic)
            self.nav_stop_topic = topics.get("nav_stop_topic", self.nav_stop_topic)
            self.nav_done_topic = topics.get("nav_done_topic", self.nav_done_topic)
            self.external_goal_topic = topics.get("external_goal_topic", self.external_goal_topic)
            self.initialpose_frame = topics.get("initialpose_frame", self.initialpose_frame)

            self.set_status("已读取系统参数配置")

    # -----------------------------------------------------
    # 导航
    # -----------------------------------------------------
    def start_route_navigation(self):
        with self.lock:
            if not self.localized:
                raise RuntimeError("尚未定位")
            if len(self.route_polyline) < 2:
                raise RuntimeError("尚未收到路线")

            self.is_auto_moving = True
            self.is_obstacle_paused = False
            self.stage = "path_tracking"
            self.nav_progress_idx = 0
            self.lookahead_target = None

            self.original_goal = dict(self.route_polyline[-1])
            self.obstacle_paused_at = 0.0
            self.replan_count = 0
            self._replan_flag = False

            self.set_status(f"开始导航，路线点数 {len(self.route_polyline)}")

    def stop_auto_move(self):
        with self.lock:
            self._reset_navigation_state_locked()
            self.set_status("导航已停止")

    def emergency_stop(self):
        with self.lock:
            self._reset_navigation_state_locked()
            self.set_status("紧急停止")

    def update_pose(self, x, y, yaw_deg, z=0.0):
        with self.lock:
            self.current_pose = {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "yaw_deg": float(yaw_deg),
            }
            self.localized = True

    def set_external_goal_request(self, goal):
        with self.lock:
            self.external_goal_seq += 1
            self.external_goal_request = {
                "seq": self.external_goal_seq,
                "goal": goal,
                "time": time.time(),
            }
            self.set_status(
                f"收到外部目标点: ({goal.get('x', 0.0):.2f}, {goal.get('y', 0.0):.2f}, {goal.get('z', 0.0):.2f})"
            )

    def consume_external_goal_request(self):
        with self.lock:
            req = self.external_goal_request
            self.external_goal_request = None
            return req

    def update_controller_state(self, data):
        if not isinstance(data, dict):
            return
        with self.lock:
            was_moving = self.is_auto_moving
            was_paused = self.is_obstacle_paused
            self.current_vx = float(data.get("current_vx", self.current_vx))
            self.current_wz = float(data.get("current_wz", self.current_wz))
            self.is_auto_moving = bool(data.get("is_auto_moving", self.is_auto_moving))
            self.stage = str(data.get("stage", self.stage))
            self.is_obstacle_paused = bool(data.get("is_obstacle_paused", self.is_obstacle_paused))
            self.nav_progress_idx = int(data.get("nav_progress_idx", self.nav_progress_idx))
            self.final_target_yaw = float(data.get("final_target_yaw", self.final_target_yaw))
            target = data.get("lookahead_target", None)
            self.lookahead_target = target if isinstance(target, dict) else None
            status = data.get("status_text", None)
            if isinstance(status, str) and status:
                self.status_text = status
            self.obstacle_distance = float(data.get("obstacle_distance", self.obstacle_distance))
            boxes_raw = data.get("obstacle_boxes", None)
            if isinstance(boxes_raw, list):
                self.obstacle_boxes = boxes_raw
            clusters_raw = data.get("obstacle_clusters", None)
            if isinstance(clusters_raw, list):
                self.obstacle_clusters = clusters_raw

            self._ctrl_seq = getattr(self, '_ctrl_seq', 0) + 1
            if self._ctrl_seq % 50 == 1:
                print(f"[CORE ctrl #{self._ctrl_seq}] obs_dist={self.obstacle_distance:.1f} "
                      f"clusters={len(self.obstacle_clusters)} boxes={len(self.obstacle_boxes)}", flush=True)

            route_pts = int(data.get("route_total_points", -1))
            if was_moving and not self.is_auto_moving and route_pts == 0:
                self._handle_navigation_completed_locked()

            if self.is_obstacle_paused and not was_paused:
                self.obstacle_paused_at = time.time()
            elif not self.is_obstacle_paused:
                self.obstacle_paused_at = 0.0

    # -----------------------------------------------------
    # 动态绕障
    # -----------------------------------------------------
    def check_replan_needed(self):
        with self.lock:
            if not self.is_obstacle_paused or self.obstacle_paused_at <= 0.0:
                return False
            now = time.time()
            if now - self.obstacle_paused_at < self.replan_wait_sec:
                return False
            if now - self.last_replan_at < self.replan_cooldown_sec:
                return False
            if self.replan_count >= self.replan_max_count:
                return False
            if self.original_goal is None:
                return False
            if self.current_pose is None:
                return False
            if not self.drivable_areas:
                return False
            return True

    def perform_replan(self):
        import copy
        try:
            from indoor_route_nav import _planning_core
        except ImportError:
            _planning_core = None

        if _planning_core is None:
            self.set_status("[绕障] C++规划模块未加载")
            return None

        with self.lock:
            start = dict(self.current_pose)
            goal = dict(self.original_goal)
            areas = copy.deepcopy(self.drivable_areas)
            blocked = [dict(b) for b in self.obstacle_boxes]
            self.replan_count += 1
            self.last_replan_at = time.time()

        try:
            pts = _planning_core.plan_path(
                start, goal, areas,
                self.route_sample_gap, 0.15,
                blocked_zones=blocked)
        except Exception as e:
            self.set_status(f"[绕障] 第{self.replan_count}次规划失败: {e}")
            return None

        if not pts or len(pts) < 2:
            self.set_status(f"[绕障] 第{self.replan_count}次规划未找到可绕行路径")
            return None

        with self.lock:
            self.route_polyline = [
                {"x": float(p["x"]), "y": float(p["y"]), "z": float(p.get("z", 0.0))}
                for p in pts
            ]
            self._bump_route_rev()

        self.set_status(
            f"[绕障] 第{self.replan_count}次重规划成功，新路线 {len(pts)} 点，"
            f"距离{self.obstacle_distance:.1f}m"
        )
        return pts

    # -----------------------------------------------------
    # 状态导出
    # -----------------------------------------------------
    def export_state(self):
        with self.lock:
            return {
                "pcd_path": self.pcd_path,
                "points_xyz": self.points_xyz,
                "points_rgb": self.points_rgb,
                "points_xy": self.points_xy,  # 兼容旧版

                "route_polyline": self.route_polyline,
                "stop_points": [],
                "drivable_areas": self.drivable_areas,
                "connectors": self.connectors,

                # 兼容旧前端保留空字段
                "route_start_anchor": dict(self.route_polyline[0]) if len(self.route_polyline) > 0 else None,
                "route_segments": [],
                "no_obstacle_zones": [],

                "mode": self.mode,
                "status_text": self.status_text,
                "localized": self.localized,
                "current_pose": self.current_pose,
                "current_vx": self.current_vx,
                "current_wz": self.current_wz,
                "obstacle_distance": self.obstacle_distance,
                "obstacle_boxes": self.obstacle_boxes,
                "obstacle_clusters": self.obstacle_clusters,

                "nav": {
                    "is_auto_moving": self.is_auto_moving,
                    "stage": self.stage,
                    "is_obstacle_paused": self.is_obstacle_paused,
                    "nav_progress_idx": self.nav_progress_idx,
                    "route_total_points": len(self.route_polyline),
                    "lookahead_target": self.lookahead_target,
                    "final_target_yaw": self.final_target_yaw,
                },

                "camera": {
                    "topic": self.camera_topic,
                    "has_frame": self.camera_frame_jpeg is not None,
                    "frame_age": (time.time() - self.camera_frame_time) if self.camera_frame_time > 0 else None,
                },

                "control_params": self.get_control_params(),
                "smoothing_params": self.get_smoothing_params(),

                "topics": {
                    "camera_topic": self.camera_topic,
                    "nav_start_topic": self.nav_start_topic,
                    "nav_stop_topic": self.nav_stop_topic,
                    "nav_done_topic": self.nav_done_topic,
                    "external_goal_topic": self.external_goal_topic,
                    "initialpose_frame": self.initialpose_frame,
                },

                "nav_start_topic": self.nav_start_topic,
                "nav_stop_topic": self.nav_stop_topic,
                "nav_done_topic": self.nav_done_topic,
                "external_goal_topic": self.external_goal_topic,

                "revisions": {
                    "map": self.map_revision,
                    "route": self.route_revision,
                    "area": self.area_revision,
                },

                "map_is_3d": True,
            }

    def export_compact_state(self):
        with self.lock:
            skip_obs = not self.use_obstacle_avoidance
            return {
                "type": "compact_state",
                "status_text": self.status_text,
                "localized": self.localized,
                "current_pose": dict(self.current_pose) if self.current_pose is not None else None,
                "current_vx": self.current_vx,
                "current_wz": self.current_wz,
                "obstacle_distance": 999.0 if skip_obs else self.obstacle_distance,
                "obstacle_boxes": [] if skip_obs else self.obstacle_boxes,
                "obstacle_clusters": [] if skip_obs else self.obstacle_clusters,
                "nav": {
                    "is_auto_moving": self.is_auto_moving,
                    "stage": self.stage,
                    "is_obstacle_paused": self.is_obstacle_paused,
                    "nav_progress_idx": self.nav_progress_idx,
                    "route_total_points": len(self.route_polyline),
                    "lookahead_target": dict(self.lookahead_target) if self.lookahead_target is not None else None,
                    "final_target_yaw": self.final_target_yaw,
                },
                "camera": {
                    "topic": self.camera_topic,
                },
                "pcd_path": self.pcd_path,
                "route_polyline": self.route_polyline,
                "route_count": len(self.route_polyline),
                "area_count": len(self.drivable_areas),
                "connector_count": len(self.connectors),
                "revisions": {
                    "map": self.map_revision,
                    "route": self.route_revision,
                    "area": self.area_revision,
                },
                "topics": {
                    "camera_topic": self.camera_topic,
                    "external_route_topic": self.external_route_topic,
                    "external_route_rich_topic": self.external_route_rich_topic,
                    "nav_start_topic": self.nav_start_topic,
                    "nav_stop_topic": self.nav_stop_topic,
                    "nav_done_topic": self.nav_done_topic,
                    "nav_clear_topic": self.nav_clear_topic,
                    "external_goal_topic": self.external_goal_topic,
                    "initialpose_frame": self.initialpose_frame,
                },
            }


# =========================================================
# ROS2 节点
# =========================================================
class WebRosBridgeNode(Node):
    def __init__(self, core: AppCore):
        super().__init__("indoor_route_nav_web")
        self.core = core
        self.bridge = CvBridge() if CvBridge is not None else None
        self.localization_seq = 0

        # 默认 QoS (RELIABLE) 与 /localization 话题发布者匹配
        # 之前 BEST_EFFORT 与 RELIABLE 发布者不兼容，导致收不到定位数据

        # 订阅 /localization 话题获取当前位置信息
        self.sub_localization = self.create_subscription(
            Odometry, "/localization", self.localization_callback, 10
        )

        self.sub_nav_start = self.create_subscription(
            Empty, self.core.nav_start_topic, self.nav_start_callback, 10
        )

        self.sub_nav_stop = self.create_subscription(
            Empty, self.core.nav_stop_topic, self.nav_stop_callback, 10
        )

        self.sub_external_goal = self.create_subscription(
            PoseStamped, self.core.external_goal_topic, self.external_goal_callback, 10
        )

        self.sub_controller_state = self.create_subscription(
            String, "/indoor_route_nav/controller/state", self.controller_state_callback, 10
        )

        self.sub_nav_done = self.create_subscription(
            String, self.core.nav_done_topic, self.nav_done_callback, 10
        )

        self.sub_camera = None
        if self.bridge is not None and cv2 is not None:
            try:
                self.sub_camera = self.create_subscription(
                    Image, self.core.camera_topic, self.camera_callback, 10
                )
            except Exception as e:
                self.get_logger().warning(f"camera subscription create failed: {e}")

        self.pub_nav_done = self.create_publisher(String, self.core.nav_done_topic, 10)
        self.pub_controller_route = self.create_publisher(Path, "/indoor_route_nav/controller/route", 10)
        self.pub_controller_config = self.create_publisher(String, "/indoor_route_nav/controller/config", 10)
        self.pub_controller_start = self.create_publisher(Empty, "/indoor_route_nav/controller/start", 10)
        self.pub_controller_stop = self.create_publisher(Empty, "/indoor_route_nav/controller/stop", 10)
        self.pub_controller_clear = self.create_publisher(Empty, "/indoor_route_nav/controller/clear", 10)
        self.pub_initialpose = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)

        self.pub_chassis_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_chassis_mode = self.create_publisher(String, "/chassis/mode", 10)
        self.pub_chassis_sitdown = self.create_publisher(Empty, "/chassis/sitdown", 10)
        self.pub_chassis_emgy_stop = self.create_publisher(Empty, "/chassis/emgy_stop", 10)
        self.pub_chassis_imu_enable = self.create_publisher(String, "/chassis/imu_enable", 10)

        self.replan_timer = self.create_timer(0.5, self.replan_timer_callback)

        self.get_logger().info("WebRosBridgeNode 启动完成")

    def get_yaw_deg_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    @staticmethod
    def yaw_deg_to_quaternion(yaw_deg):
        yaw = math.radians(yaw_deg)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return {
            "x": 0.0,
            "y": 0.0,
            "z": sy,
            "w": cy,
        }

    def publish_initial_pose(self, x, y, z, yaw_deg):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.core.initialpose_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        q = self.yaw_deg_to_quaternion(float(yaw_deg))
        msg.pose.pose.orientation.x = q["x"]
        msg.pose.pose.orientation.y = q["y"]
        msg.pose.pose.orientation.z = q["z"]
        msg.pose.pose.orientation.w = q["w"]
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942,
        ]
        self.pub_initialpose.publish(msg)
        self.get_logger().info(
            f"published /initialpose ({x:.2f}, {y:.2f}, {z:.2f}, yaw={yaw_deg:.1f}°)")

    def publish_chassis_cmd_vel(self, vx, vy, vz):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(vz)
        self.pub_chassis_cmd_vel.publish(msg)

    def publish_chassis_mode(self, mode, enable=True):
        payload = {"mode": mode, "enable": enable}
        if mode not in ("stair",):
            payload.pop("enable", None)
        self.pub_chassis_mode.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f"chassis mode: {mode} (enable={enable})")

    def publish_chassis_sitdown(self):
        self.pub_chassis_sitdown.publish(Empty())
        self.get_logger().info("chassis sitdown")

    def publish_chassis_emgy_stop(self):
        self.pub_chassis_emgy_stop.publish(Empty())
        self.get_logger().info("chassis emergency stop")

    def publish_chassis_imu_enable(self, enable):
        payload = {"enable": bool(enable)}
        self.pub_chassis_imu_enable.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f"chassis IMU enable={enable}")

    def update_core_pose_from_ros_pose(self, pose):
        try:
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            yaw = self.get_yaw_deg_from_quaternion(pose.orientation)

            was_localized = self.core.localized
            self.core.update_pose(x, y, yaw, z)
            if not was_localized:
                self.core.set_status("已定位，可以导航")
        except Exception as e:
            self.get_logger().error(f"update_core_pose_from_ros_pose error: {e}")

    def localization_callback(self, msg: Odometry):
        try:
            self.update_core_pose_from_ros_pose(msg.pose.pose)
            if self.core.localized and not self.core.status_text.startswith("已定位"):
                self.core.set_status("已定位，可以导航")
        except Exception as e:
            self.get_logger().error(f"localization_callback error: {e}")

    def camera_callback(self, msg: Image):
        if self.bridge is None or cv2 is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                self.core.update_camera_frame(enc.tobytes())
        except Exception as e:
            self.get_logger().warning(f"camera_callback error: {e}")

    def external_route_callback(self, msg: Path):
        try:
            pts = []
            for p in msg.poses:
                pts.append({
                    "x": float(p.pose.position.x),
                    "y": float(p.pose.position.y),
                    "z": float(p.pose.position.z),
                })

            if len(pts) >= 2:
                self.core.set_external_route(pts, auto_start=False)
                self.publish_controller_route()
                self.publish_controller_config()
        except Exception as e:
            self.get_logger().error(f"external_route_callback error: {e}")

    def external_route_rich_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.core.set_external_route_bundle(data)
            self.publish_controller_route()
            self.publish_controller_config()
            if bool(data.get("auto_start", False)):
                self.pub_controller_start.publish(Empty())
        except Exception as e:
            self.core.set_status(f"外部路线包读取失败: {e}")
            self.get_logger().error(f"external_route_rich_callback error: {e}")

    def external_goal_callback(self, msg: PoseStamped):
        try:
            goal = {
                "x": float(msg.pose.position.x),
                "y": float(msg.pose.position.y),
                "z": float(msg.pose.position.z),
                "yaw_deg": self.get_yaw_deg_from_quaternion(msg.pose.orientation),
            }
            self.core.set_external_goal_request(goal)
        except Exception as e:
            self.get_logger().error(f"external_goal_callback error: {e}")

    def controller_state_callback(self, msg: String):
        try:
            self.core.update_controller_state(json.loads(msg.data))
        except Exception as e:
            self.get_logger().error(f"controller_state_callback error: {e}")

    def replan_timer_callback(self):
        if not self.core.check_replan_needed():
            return
        pts = self.core.perform_replan()
        if pts is not None and len(pts) >= 2:
            self.publish_controller_route()
            self.publish_controller_config()
            self.pub_controller_start.publish(Empty())

    def nav_start_callback(self, _msg: Empty):
        try:
            self.core.start_route_navigation()
            self.publish_controller_route()
            self.publish_controller_config()
            self.pub_controller_start.publish(Empty())
        except Exception as e:
            self.core.set_status(f"开启导航失败: {e}")
            self.get_logger().error(f"nav_start_callback error: {e}")

    def nav_stop_callback(self, _msg: Empty):
        try:
            self.stop_controller_navigation()
            self.core.set_status("已通过外部话题停止导航")
        except Exception as e:
            self.core.set_status(f"停止导航失败: {e}")
            self.get_logger().error(f"nav_stop_callback error: {e}")

    def nav_clear_callback(self, _msg: Empty):
        try:
            self.core.clear_navigation_data()
            self.pub_controller_clear.publish(Empty())
        except Exception as e:
            self.get_logger().error(f"nav_clear_callback error: {e}")

    def publish_controller_route(self):
        with self.core.lock:
            route = [dict(p) for p in self.core.route_polyline]
            frame_id = self.core.initialpose_frame
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        for p in route:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(p.get("x", 0.0))
            pose.pose.position.y = float(p.get("y", 0.0))
            pose.pose.position.z = float(p.get("z", 0.0))
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.pub_controller_route.publish(path)

    def publish_controller_config(self):
        with self.core.lock:
            data = self.core.get_control_params()
            data["final_target_yaw"] = self.core.final_target_yaw
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.pub_controller_config.publish(msg)

    def start_controller_navigation(self):
        self.core.start_route_navigation()
        self.publish_controller_route()
        self.publish_controller_config()
        self.pub_controller_start.publish(Empty())

    def stop_controller_navigation(self):
        self.core.stop_auto_move()
        self.pub_controller_stop.publish(Empty())

    def nav_done_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("event") == "nav_done" and data.get("success"):
                self.get_logger().info("收到 nav_done，结束导航任务")
                self.pub_controller_stop.publish(Empty())
                self.pub_controller_clear.publish(Empty())
                with self.core.lock:
                    self.core._handle_navigation_completed_locked()
        except Exception as e:
            self.get_logger().error(f"nav_done_callback error: {e}")

    def emergency_controller_stop(self):
        self.core.emergency_stop()
        self.pub_controller_stop.publish(Empty())

    def clear_controller_navigation(self):
        self.core.clear_navigation_data()
        self.pub_controller_clear.publish(Empty())


# =========================================================
# ROS 运行器
# =========================================================
class RosRunner:
    def __init__(self, core: AppCore):
        self.core = core
        self.node = None
        self.thread = None
        self.started = False

    def start(self):
        if self.started:
            return
        rclpy.init(args=None)
        self.node = WebRosBridgeNode(self.core)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()
        self.started = True

    def stop(self):
        if not self.started:
            return

        try:
            self.core.emergency_stop()
        except Exception:
            pass

        try:
            if self.node is not None:
                self.node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass

        self.started = False

    def sync_controller_route(self):
        if not self.started or self.node is None:
            return
        self.node.publish_controller_route()
        self.node.publish_controller_config()

    def sync_controller_config(self):
        if not self.started or self.node is None:
            return
        self.node.publish_controller_config()

    def publish_initial_pose(self, x, y, z, yaw_deg):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_initial_pose(x, y, z, yaw_deg)

    def start_navigation(self):
        if not self.started or self.node is None:
            self.core.start_route_navigation()
            return
        self.node.start_controller_navigation()

    def stop_navigation(self):
        if not self.started or self.node is None:
            self.core.stop_auto_move()
            return
        self.node.stop_controller_navigation()

    def emergency_stop_navigation(self):
        if not self.started or self.node is None:
            self.core.emergency_stop()
            return
        self.node.emergency_controller_stop()

    def clear_navigation(self):
        if not self.started or self.node is None:
            self.core.clear_navigation_data()
            return
        self.node.clear_controller_navigation()

    def chassis_cmd_vel(self, vx, vy, vz):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_chassis_cmd_vel(vx, vy, vz)

    def chassis_mode(self, mode, enable=True):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_chassis_mode(mode, enable)

    def chassis_sitdown(self):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_chassis_sitdown()

    def chassis_emgy_stop(self):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_chassis_emgy_stop()

    def chassis_imu_enable(self, enable):
        if not self.started or self.node is None:
            raise RuntimeError("ROS节点未启动")
        self.node.publish_chassis_imu_enable(enable)


# =========================================================
# 全局对象
# =========================================================
core = AppCore()
ros_runner = RosRunner(core)