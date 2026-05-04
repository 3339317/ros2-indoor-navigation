#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import io
import time
import json
import yaml
import socket
import signal
import tempfile
import threading
import subprocess
import shutil
import asyncio
import copy
import math

import numpy as np

try:
    from indoor_route_nav import _planning_core
except Exception:
    _planning_core = None

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

from fastapi import FastAPI, UploadFile, File, Body, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

from core_ros import core, ros_runner

APP_PORT = 8000
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
if not os.path.isdir(STATIC_DIR) and get_package_share_directory is not None:
    try:
        STATIC_DIR = os.path.join(get_package_share_directory("indoor_route_nav"), "static")
    except Exception:
        pass
INDEX_FILE = os.path.join(STATIC_DIR, "index.html")
DEFAULT_CONFIG_FILE = os.path.join(BASE_DIR, "config", "default_params.yaml")
MAP_PREPROCESSOR_CONFIG_FILE = os.path.join(BASE_DIR, "config", "map_preprocessor.yaml")
if get_package_share_directory is not None:
    try:
        share_dir = get_package_share_directory("indoor_route_nav")
        share_default_config = os.path.join(share_dir, "config", "default_params.yaml")
        if os.path.exists(share_default_config):
            DEFAULT_CONFIG_FILE = share_default_config
        share_preprocessor_config = os.path.join(share_dir, "config", "map_preprocessor.yaml")
        if os.path.exists(share_preprocessor_config):
            MAP_PREPROCESSOR_CONFIG_FILE = share_preprocessor_config
    except Exception:
        pass

app = FastAPI(title="ROS2 Web Route Nav 3D")
external_goal_worker_stop = threading.Event()
external_goal_worker_thread = None

if os.path.isdir(STATIC_DIR):
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


# =========================================================
# 工具
# =========================================================
def ensure_parent_dir(file_path: str):
    parent = os.path.dirname(os.path.abspath(file_path))
    if parent and not os.path.exists(parent):
        os.makedirs(parent, exist_ok=True)


def build_topics_dict():
    return {
        "camera_topic": getattr(core, "camera_topic", ""),
        "nav_start_topic": getattr(core, "nav_start_topic", ""),
        "nav_stop_topic": getattr(core, "nav_stop_topic", ""),
        "nav_done_topic": getattr(core, "nav_done_topic", ""),
        "external_goal_topic": getattr(core, "external_goal_topic", ""),
        "initialpose_frame": getattr(core, "initialpose_frame", "map"),
    }


def export_full_state():
    if hasattr(core, "export_state"):
        state = core.export_state()
    else:
        state = {}

    if not isinstance(state, dict):
        state = {}

    if "points_xyz" not in state:
        pts_xy = state.get("points_xy", [])
        pts_xyz = []
        try:
            for p in pts_xy:
                pts_xyz.append([float(p[0]), float(p[1]), 0.0])
        except Exception:
            pts_xyz = []
        state["points_xyz"] = pts_xyz

    if "points_rgb" not in state:
        state["points_rgb"] = []

    if "topics" not in state or not isinstance(state["topics"], dict):
        state["topics"] = {}

    topics = state["topics"]
    for k, v in build_topics_dict().items():
        topics.setdefault(k, v)
    state["topics"] = topics

    if "camera" not in state or not isinstance(state["camera"], dict):
        state["camera"] = {"topic": topics.get("camera_topic", "")}
    else:
        state["camera"].setdefault("topic", topics.get("camera_topic", ""))

    if "nav" not in state or not isinstance(state["nav"], dict):
        state["nav"] = {}

    state.setdefault("map_is_3d", True)

    if "revisions" not in state:
        state["revisions"] = {
            "map": getattr(core, "map_revision", 0),
            "route": getattr(core, "route_revision", 0),
            "area": getattr(core, "area_revision", 0),
        }
    else:
        state["revisions"].setdefault("area", getattr(core, "area_revision", 0))

    return state


def build_compact_state():
    if hasattr(core, "export_compact_state"):
        try:
            st = core.export_compact_state()
            if isinstance(st, dict):
                st["topics"] = st.get("topics", build_topics_dict())
                st["type"] = "compact_state"
                if "revisions" not in st:
                    st["revisions"] = {
                        "map": getattr(core, "map_revision", 0),
                        "route": getattr(core, "route_revision", 0),
                        "area": getattr(core, "area_revision", 0),
                    }
                return st
        except Exception as e:
            import traceback
            print(f"[build_compact_state] export_compact_state ERROR: {e}", flush=True)
            traceback.print_exc()

    print("[build_compact_state] FALLBACK path used", flush=True)

    full = export_full_state()
    return {
        "type": "compact_state",
        "status_text": full.get("status_text", "等待操作"),
        "localized": full.get("localized", False),
        "current_pose": full.get("current_pose"),
        "current_vx": full.get("current_vx", 0.0),
        "current_wz": full.get("current_wz", 0.0),
        "obstacle_distance": full.get("obstacle_distance", 999.0),
        "obstacle_boxes": full.get("obstacle_boxes", []),
        "obstacle_clusters": full.get("obstacle_clusters", []),
        "nav": full.get("nav", {}),
        "camera": full.get("camera", {}),
        "pcd_path": full.get("pcd_path", ""),
        "route_count": len(full.get("route_polyline", []) or []),
        "route_polyline": full.get("route_polyline", []),
        "topics": full.get("topics", build_topics_dict()),
        "revisions": full.get("revisions", {
            "map": 0, "route": 0, "area": 0
        }),
    }


def yaw_deg_to_quaternion(yaw_deg):
    yaw = math.radians(float(yaw_deg))
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return {"x": 0.0, "y": 0.0, "z": sy, "w": cy}


def plan_path_native(start, goal, areas, resolution, robot_radius):
    if _planning_core is None:
        raise RuntimeError("C++规划模块未加载，请重新编译")
    pts = _planning_core.plan_path(start, goal, areas, resolution, robot_radius)
    if not pts or len(pts) < 2:
        raise RuntimeError("C++规划结果点数不足")
    return pts


def apply_route_points(points):
    if hasattr(core, "set_route_points"):
        core.set_route_points(points)
        ros_runner.sync_controller_route()
        return
    if hasattr(core, "set_external_route"):
        core.set_external_route(points)
        ros_runner.sync_controller_route()
        return
    raise RuntimeError("core 不支持设置路线")


def _point_like_to_xyz(item):
    if isinstance(item, dict):
        if "x" in item and "y" in item:
            return {
                "x": float(item["x"]),
                "y": float(item["y"]),
                "z": float(item.get("z", 0.0)),
            }

        if "position" in item and isinstance(item["position"], dict):
            pos = item["position"]
            return {
                "x": float(pos["x"]),
                "y": float(pos["y"]),
                "z": float(pos.get("z", 0.0)),
            }

        if "pose" in item and isinstance(item["pose"], dict):
            pose = item["pose"]
            if "position" in pose and isinstance(pose["position"], dict):
                pos = pose["position"]
                return {
                    "x": float(pos["x"]),
                    "y": float(pos["y"]),
                    "z": float(pos.get("z", 0.0)),
                }
            if "x" in pose and "y" in pose:
                return {
                    "x": float(pose["x"]),
                    "y": float(pose["y"]),
                    "z": float(pose.get("z", 0.0)),
                }

    if isinstance(item, (list, tuple, np.ndarray)):
        arr = np.asarray(item).reshape(-1)
        if arr.size >= 2:
            return {
                "x": float(arr[0]),
                "y": float(arr[1]),
                "z": float(arr[2]) if arr.size >= 3 else 0.0,
            }

    raise RuntimeError(f"无法解析点: {item}")


def _extract_points_from_ndarray(arr):
    arr = np.asarray(arr)

    if arr.dtype == object:
        if arr.ndim == 0:
            return _extract_points_from_any(arr.item())
        return _extract_points_from_any(arr.tolist())

    if arr.ndim == 2 and arr.shape[1] >= 2:
        pts = []
        for row in arr:
            pts.append({
                "x": float(row[0]),
                "y": float(row[1]),
                "z": float(row[2]) if arr.shape[1] >= 3 else 0.0,
            })
        if len(pts) >= 2:
            return pts

    raise RuntimeError("ndarray中未找到可用路线点")


def _extract_points_from_any(obj):
    search_keys = [
        "points",
        "route_points",
        "route_polyline",
        "path",
        "trajectory",
        "polyline",
        "merged_result",
        "merged_points",
        "result",
    ]

    if isinstance(obj, np.ndarray):
        return _extract_points_from_ndarray(obj)

    if isinstance(obj, dict):
        candidates = []

        if "route" in obj:
            rv = obj["route"]
            if isinstance(rv, dict):
                for k in search_keys:
                    if k in rv:
                        candidates.append(rv[k])
            else:
                candidates.append(rv)

        for k in search_keys:
            if k in obj:
                candidates.append(obj[k])

        if "poses" in obj and isinstance(obj["poses"], list):
            candidates.append(obj["poses"])

        for cand in candidates:
            try:
                pts = _extract_points_from_any(cand)
                if len(pts) >= 2:
                    return pts
            except Exception:
                pass

        raise RuntimeError("文件中未找到可用路线点")

    if isinstance(obj, (list, tuple)):
        if len(obj) == 0:
            raise RuntimeError("路线为空")
        pts = []
        for item in obj:
            pts.append(_point_like_to_xyz(item))
        if len(pts) < 2:
            raise RuntimeError("路线点数不足")
        return pts

    raise RuntimeError("不支持的路线数据格式")


def parse_route_json_bytes(content: bytes):
    data = json.loads(content.decode("utf-8"))
    return _extract_points_from_any(data)


def parse_route_npz_bytes(content: bytes):
    bio = io.BytesIO(content)
    npz = np.load(bio, allow_pickle=True)

    candidate_keys = [
        "points",
        "route_points",
        "route_polyline",
        "path",
        "trajectory",
        "polyline",
        "merged_result",
        "merged_points",
        "result",
        "arr_0",
    ]

    for k in candidate_keys:
        if k in npz.files:
            try:
                pts = _extract_points_from_any(npz[k])
                if len(pts) >= 2:
                    return pts
            except Exception:
                pass

    for k in npz.files:
        try:
            pts = _extract_points_from_any(npz[k])
            if len(pts) >= 2:
                return pts
        except Exception:
            pass

    raise RuntimeError("NPZ中未找到可用路线点")


def parse_structured_upload(content: bytes, filename: str):
    ext = os.path.splitext(filename)[1].lower()
    text = content.decode("utf-8")

    if ext == ".json":
        return json.loads(text)

    try:
        return yaml.safe_load(text) or {}
    except Exception:
        return json.loads(text)


def load_default_system_params():
    if not os.path.exists(DEFAULT_CONFIG_FILE):
        return
    with open(DEFAULT_CONFIG_FILE, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if isinstance(data, dict):
        core.load_system_params_data(data)


def find_map_preprocessor_executable():
    candidates = [
        os.path.join(BASE_DIR, "indoor_map_preprocessor_node"),
        os.path.join(
            os.path.expanduser("~"),
            "indoor_nav_ws",
            "install",
            "indoor_route_nav",
            "lib",
            "indoor_route_nav",
            "indoor_map_preprocessor_node"
        ),
    ]
    found = shutil.which("indoor_map_preprocessor_node")
    if found:
        candidates.append(found)

    for path in candidates:
        if path and os.path.exists(path) and os.access(path, os.X_OK):
            return path

    ros2 = shutil.which("ros2")
    if ros2:
        return None
    raise RuntimeError("找不到 indoor_map_preprocessor_node，请先 colcon build 并 source install/setup.bash")


def build_preprocessor_config(input_path, output_path, params):
    stair = params.get("stairs", {}) if isinstance(params.get("stairs", {}), dict) else {}
    floors = params.get("floors", {}) if isinstance(params.get("floors", {}), dict) else {}
    point_cloud = params.get("point_cloud", {}) if isinstance(params.get("point_cloud", {}), dict) else {}

    return {
        "input_path": input_path,
        "output_path": output_path,
        "map": {
            "name": str(params.get("map_name", "web_current_map")),
            "frame_id": getattr(core, "initialpose_frame", "map"),
        },
        "point_cloud": {
            "sample_step": 1,
            "voxel_leaf_size": float(point_cloud.get("voxel_leaf_size", params.get("voxel_leaf_size", 0.03))),
        },
        "floors": {
            "mode": "auto",
            "histogram_bin_size": float(floors.get("histogram_bin_size", params.get("floor_histogram_bin_size", 0.10))),
            "min_peak_points": int(floors.get("min_peak_points", params.get("floor_min_peak_points", 800))),
            "min_floor_gap": float(floors.get("min_floor_gap", params.get("floor_min_gap", 1.20))),
            "floor_thickness": float(floors.get("floor_thickness", params.get("floor_thickness", 0.45))),
            "manual_ranges": [],
        },
        "drivable_area": {
            "generate": False,
            "boundary_margin": 0.20,
            "percentile_low": 2.0,
            "percentile_high": 98.0,
            "min_points_per_area": 1000,
        },
        "stairs": {
            "generate": True,
            "grid_resolution": float(stair.get("grid_resolution", params.get("stair_grid_resolution", 0.20))),
            "min_cell_points": int(stair.get("min_cell_points", params.get("stair_min_cell_points", 1))),
            "min_component_cells": int(stair.get("min_component_cells", params.get("stair_min_component_cells", 8))),
            "floor_exclusion": float(stair.get("floor_exclusion", params.get("stair_floor_exclusion", 0.25))),
            "min_vertical_span": float(stair.get("min_vertical_span", params.get("stair_min_vertical_span", 0.60))),
            "min_floor_coverage": float(stair.get("min_floor_coverage", params.get("stair_min_floor_coverage", 0.40))),
            "min_length": float(stair.get("min_length", params.get("stair_min_length", 0.80))),
            "max_width": float(stair.get("max_width", params.get("stair_max_width", 4.00))),
            "polyline_points": int(stair.get("polyline_points", params.get("stair_polyline_points", 8))),
        },
        "metadata": {
            "robot_radius": float(params.get("robot_radius", 0.25)),
            "grid_resolution": float(params.get("grid_resolution", 0.05)),
        },
    }


def write_current_points_xyz(file_path):
    with core.lock:
        raw_points = list(getattr(core, "points_xyz", []) or [])
    points = [_point_like_to_xyz(p) for p in raw_points]
    if len(points) == 0:
        raise RuntimeError("当前没有点云地图")
    with open(file_path, "w", encoding="utf-8") as f:
        for p in points:
            f.write(f"{float(p.get('x', 0.0))} {float(p.get('y', 0.0))} {float(p.get('z', 0.0))}\n")
    return len(points)


def run_map_preprocessor(config_path):
    exe = find_map_preprocessor_executable()
    if exe is None:
        cmd = [
            "ros2", "run", "indoor_route_nav", "indoor_map_preprocessor_node",
            "--ros-args", "-p", f"config_path:={config_path}"
        ]
    else:
        cmd = [exe, "--config", config_path]

    return subprocess.run(
        cmd,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=120,
    )


def reapply_route_sampling_if_possible():
    if not hasattr(core, "resample_polyline"):
        return
    if not hasattr(core, "build_route_cumlen"):
        return
    if not hasattr(core, "route_polyline"):
        return

    with core.lock:
        route = getattr(core, "route_polyline", []) or []
        if len(route) < 2:
            return

        base = [dict(p) for p in route]
        core.route_polyline = core.resample_polyline(base, core.route_sample_gap)
        core.route_cumlen = core.build_route_cumlen(core.route_polyline)

        if hasattr(core, "_bump_route_rev"):
            core._bump_route_rev()

        core.set_status("已应用路径采样参数")


def point_in_polygon_xy(x, y, polygon):
    inside = False
    n = len(polygon)
    if n < 3:
        return False

    j = n - 1
    for i in range(n):
        xi = float(polygon[i]["x"])
        yi = float(polygon[i]["y"])
        xj = float(polygon[j]["x"])
        yj = float(polygon[j]["y"])

        if ((yi > y) != (yj > y)):
            x_cross = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < x_cross:
                inside = not inside
        j = i

    return inside


def dist_point_to_segment_xy(px, py, ax, ay, bx, by):
    abx = bx - ax
    aby = by - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / ab2
    t = max(0.0, min(1.0, t))
    qx = ax + t * abx
    qy = ay + t * aby
    return math.hypot(px - qx, py - qy)


def min_dist_to_polygon_edges_xy(x, y, polygon):
    best = 1e18
    for i in range(len(polygon)):
        a = polygon[i]
        b = polygon[(i + 1) % len(polygon)]
        best = min(
            best,
            dist_point_to_segment_xy(
                x, y,
                float(a["x"]), float(a["y"]),
                float(b["x"]), float(b["y"])
            )
        )
    return best


def point_in_polygon_or_near_edge_xy(x, y, polygon, tolerance=0.03):
    return point_in_polygon_xy(x, y, polygon) or min_dist_to_polygon_edges_xy(x, y, polygon) <= tolerance


def interpolated_z_on_segment_xy(x, y, a, b):
    ax = float(a["x"])
    ay = float(a["y"])
    az = float(a.get("z", 0.0))
    bx = float(b["x"])
    by = float(b["y"])
    bz = float(b.get("z", az))

    abx = bx - ax
    aby = by - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-12:
        return az, math.hypot(x - ax, y - ay)

    t = ((x - ax) * abx + (y - ay) * aby) / ab2
    t = max(0.0, min(1.0, t))
    qx = ax + t * abx
    qy = ay + t * aby
    return az + (bz - az) * t, math.hypot(x - qx, y - qy)


def estimate_area_surface_z(x, y, polys, fallback_z=0.0):
    best = None
    best_dist = 1e18

    for poly in polys:
        if not point_in_polygon_xy(x, y, poly):
            continue
        for i in range(len(poly)):
            z, dist = interpolated_z_on_segment_xy(x, y, poly[i], poly[(i + 1) % len(poly)])
            if dist < best_dist:
                best = z
                best_dist = dist

    if best is not None:
        return float(best)

    weighted = 0.0
    weight_sum = 0.0
    for poly in polys:
        for p in poly:
            d = math.hypot(x - float(p["x"]), y - float(p["y"]))
            w = 1.0 / max(d, 0.05)
            weighted += float(p.get("z", fallback_z)) * w
            weight_sum += w

    if weight_sum > 0.0:
        return float(weighted / weight_sum)
    return float(fallback_z)


def estimate_cloud_surface_z_at(x, y, hint_z=None, radius=0.60, z_window=1.20):
    with core.lock:
        raw_points = list(getattr(core, "points_xyz", []) or [])
    if not raw_points:
        return None

    x = float(x)
    y = float(y)
    hint = float(hint_z) if hint_z is not None else None
    radius = max(0.05, float(radius))
    z_window = max(0.05, float(z_window))

    weighted = 0.0
    weight_sum = 0.0
    best = None
    best_score = 1e18
    r2 = radius * radius
    step = max(1, int(len(raw_points) / 120000))

    for item in raw_points[::step]:
        try:
            p = _point_like_to_xyz(item)
        except Exception:
            continue
        dx = float(p["x"]) - x
        dy = float(p["y"]) - y
        d2 = dx * dx + dy * dy
        if d2 > r2:
            continue
        pz = float(p.get("z", 0.0))
        dz = abs(pz - hint) if hint is not None else 0.0
        if hint is not None and dz > z_window:
            continue
        d = math.sqrt(d2)
        w = 1.0 / max(0.03, d)
        weighted += pz * w
        weight_sum += w
        score = d + dz * 0.20
        if score < best_score:
            best = pz
            best_score = score

    if weight_sum > 0.0:
        return float(weighted / weight_sum)
    return best


def project_pose_to_cloud_surface(point, label, radius=0.80, z_window=1.20):
    p = dict(point)
    z = estimate_cloud_surface_z_at(
        p.get("x", 0.0),
        p.get("y", 0.0),
        hint_z=p.get("z", None),
        radius=radius,
        z_window=z_window,
    )
    if z is not None:
        p["z"] = float(z)
        return p, f"{label}高度已投影到脚下点云 z={z:.2f}"
    return p, f"{label}附近未找到可用点云高度，沿用 z={float(p.get('z', 0.0)):.2f}"


def areas_covering_z(areas, z):
    z = float(z)
    return [
        a for a in areas
        if float(a.get("z_min", -1e18)) <= z <= float(a.get("z_max", 1e18))
    ]


def corridor_areas_as_connectors(areas):
    connectors = []
    for i, area in enumerate(areas or []):
        if str(area.get("type", "")).lower() != "corridor":
            continue
        centerline = area.get("centerline", [])
        if not isinstance(centerline, list) or len(centerline) < 2:
            continue
        try:
            points = [_point_like_to_xyz(p) for p in centerline]
        except Exception:
            continue
        z_vals = [float(p.get("z", 0.0)) for p in points]
        connectors.append({
            "type": "corridor",
            "name": str(area.get("name", f"corridor_{i}")),
            "z_min": float(area.get("z_min", min(z_vals))),
            "z_max": float(area.get("z_max", max(z_vals))),
            "points": points,
        })
    return connectors


def unique_append_path(out, points, eps=1e-6):
    for p in points:
        pp = {"x": float(p["x"]), "y": float(p["y"]), "z": float(p.get("z", 0.0))}
        if out:
            last = out[-1]
            if (
                abs(last["x"] - pp["x"]) < eps and
                abs(last["y"] - pp["y"]) < eps and
                abs(last.get("z", 0.0) - pp["z"]) < eps
            ):
                continue
        out.append(pp)


def sorted_connector_points(connector, ascending=True):
    pts = [_point_like_to_xyz(p) for p in connector.get("points", [])]
    pts.sort(key=lambda p: float(p.get("z", 0.0)))
    if not ascending:
        pts.reverse()
    return pts


def nearest_point_on_polyline_xy(point, polyline):
    px = float(point.get("x", 0.0))
    py = float(point.get("y", 0.0))
    best = None
    for i in range(len(polyline) - 1):
        a = polyline[i]
        b = polyline[i + 1]
        ax = float(a["x"])
        ay = float(a["y"])
        az = float(a.get("z", 0.0))
        bx = float(b["x"])
        by = float(b["y"])
        bz = float(b.get("z", az))
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 < 1e-12:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
        q = {
            "x": ax + t * abx,
            "y": ay + t * aby,
            "z": az + t * (bz - az),
        }
        dist = math.hypot(px - q["x"], py - q["y"])
        if best is None or dist < best[0]:
            best = (dist, q, i, t)
    return best


def estimate_drivable_area_surface_z(area, x, y, fallback_z=0.0):
    centerline = area.get("centerline", [])
    if isinstance(centerline, list) and len(centerline) >= 2:
        near = nearest_point_on_polyline_xy({"x": x, "y": y}, centerline)
        if near is not None:
            return float(near[1].get("z", fallback_z))

    points = area.get("points", [])
    if isinstance(points, list) and len(points) >= 3:
        return float(estimate_area_surface_z(x, y, [points], fallback_z))

    return float(fallback_z)


def project_pose_to_lower_drivable_area(point, areas, label, xy_tolerance=0.12, z_tolerance=0.35):
    p = dict(point)
    x = float(p.get("x", 0.0))
    y = float(p.get("y", 0.0))
    z = float(p.get("z", 0.0))

    candidates = []
    for idx, area in enumerate(areas):
        poly = area.get("points", [])
        if not isinstance(poly, list) or len(poly) < 3:
            continue
        if not point_in_polygon_or_near_edge_xy(x, y, poly, xy_tolerance):
            continue

        fallback_z = 0.5 * (
            float(area.get("z_min", z)) + float(area.get("z_max", z))
        )
        surface_z = estimate_drivable_area_surface_z(area, x, y, fallback_z)
        if surface_z <= z + float(z_tolerance):
            candidates.append({
                "index": idx,
                "name": str(area.get("name", f"Area{idx}")),
                "surface_z": float(surface_z),
                "vertical_gap": z - float(surface_z),
            })

    if not candidates:
        return p, f"{label}下方未命中可行驶区域，沿用当前高度", False

    candidates.sort(key=lambda c: (abs(c["vertical_gap"]), -c["surface_z"]))
    chosen = candidates[0]
    p["z"] = chosen["surface_z"]
    return (
        p,
        f"{label}已落到下方可行驶区域 {chosen['name']} z={chosen['surface_z']:.2f}",
        True,
    )


def trim_connector_points_between(connector_points, start_ref, goal_ref, ascending=True):
    if len(connector_points) < 2:
        return connector_points
    a = nearest_point_on_polyline_xy(start_ref, connector_points)
    b = nearest_point_on_polyline_xy(goal_ref, connector_points)
    if a is None or b is None:
        return connector_points
    _da, pa, ia, ta = a
    _db, pb, ib, tb = b
    sa = ia + ta
    sb = ib + tb
    if sa > sb:
        pa, pb = pb, pa
        ia, ib = ib, ia

    out = [pa]
    for idx in range(ia + 1, ib + 1):
        if 0 <= idx < len(connector_points):
            out.append(connector_points[idx])
    out.append(pb)
    if not ascending:
        out.reverse()
    return out


def plan_segment_or_direct(start, goal, areas, resolution, robot_radius):
    seg_areas = areas_covering_z(areas, float(goal.get("z", start.get("z", 0.0))))
    if seg_areas:
        try:
            return plan_path_native(start, goal, seg_areas, resolution, robot_radius)
        except Exception:
            pass

    return [
        {"x": float(start["x"]), "y": float(start["y"]), "z": float(start.get("z", 0.0))},
        {"x": float(goal["x"]), "y": float(goal["y"]), "z": float(goal.get("z", start.get("z", 0.0)))},
    ]


def plan_path_with_connectors(start, goal, areas, connectors, resolution=0.20, robot_radius=0.15):
    if not connectors:
        raise RuntimeError("起点和目标点不在同一高度层，且没有可用中心线通道")

    z0 = float(start.get("z", 0.0))
    z1 = float(goal.get("z", z0))
    z_lo = min(z0, z1)
    z_hi = max(z0, z1)
    ascending = z1 >= z0

    candidates = [
        c for c in connectors
        if float(c.get("z_min", -1e18)) <= z_lo + 0.25
        and float(c.get("z_max", 1e18)) >= z_hi - 0.25
        and len(c.get("points", [])) >= 2
    ]
    if not candidates:
        raise RuntimeError("没有覆盖起点和目标楼层高度的中心线通道")

    # 先选高度覆盖最接近的连接区，避免跨越过多楼层。
    candidates.sort(key=lambda c: abs(float(c.get("z_min", z_lo)) - z_lo) + abs(float(c.get("z_max", z_hi)) - z_hi))
    connector = candidates[0]
    stair_points = sorted_connector_points(connector, ascending=ascending)
    if len(stair_points) < 2:
        raise RuntimeError("中心线通道点数不足")

    stair_points = trim_connector_points_between(stair_points, start, goal, ascending=True)
    if not ascending:
        stair_points.reverse()
    entry = stair_points[0]
    exit_pt = stair_points[-1]

    out = []
    unique_append_path(out, plan_segment_or_direct(start, entry, areas, resolution, robot_radius))
    unique_append_path(out, stair_points)
    unique_append_path(out, plan_segment_or_direct(exit_pt, goal, areas, resolution, robot_radius))
    if out:
        out[0] = {"x": float(start["x"]), "y": float(start["y"]), "z": float(start.get("z", 0.0))}
        out[-1] = {"x": float(goal["x"]), "y": float(goal["y"]), "z": float(goal.get("z", start.get("z", 0.0)))}
    return out, f"跨楼层规划完成，使用中心线通道 {connector.get('name', 'corridor')}, 路径点 {len(out)} 个"


def external_goal_worker():
    while not external_goal_worker_stop.is_set():
        try:
            req = core.consume_external_goal_request()
            if req is None:
                time.sleep(0.05)
                continue

            goal = dict(req.get("goal", {}))
            payload = {
                "goal": {
                    "x": goal.get("x", 0.0),
                    "y": goal.get("y", 0.0),
                    "z": goal.get("z", 0.0),
                },
                "final_yaw_deg": goal.get("yaw_deg", 0.0),
                "resolution": 0.20,
                "robot_radius": 0.15,
            }
            result = execute_drivable_plan_to_goal(payload)
            core.set_status(result.get("message", "外部目标规划完成"))
        except Exception as e:
            core.set_status(f"外部目标规划失败: {e}")
            time.sleep(0.1)


# =========================================================
# FastAPI 生命周期
# =========================================================
@app.on_event("startup")
async def startup_event():
    global external_goal_worker_thread
    try:
        load_default_system_params()
    except Exception as e:
        core.set_status(f"默认参数读取失败: {e}")
    ros_runner.start()
    external_goal_worker_stop.clear()
    external_goal_worker_thread = threading.Thread(target=external_goal_worker, daemon=True)
    external_goal_worker_thread.start()


@app.on_event("shutdown")
async def shutdown_event():
    external_goal_worker_stop.set()
    ros_runner.stop()


# =========================================================
# 页面
# =========================================================
@app.get("/")
async def index():
    if not os.path.exists(INDEX_FILE):
        return Response(
            content="找不到 static/index.html，请检查目录结构。",
            media_type="text/plain; charset=utf-8"
        )
    return FileResponse(
        INDEX_FILE,
        headers={
            "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
            "Pragma": "no-cache",
        },
    )


# =========================================================
# 状态
# =========================================================
@app.get("/api/state")
async def get_state():
    return export_full_state()


@app.get("/api/state/compact")
async def get_state_compact():
    return build_compact_state()


# =========================================================
# 地图
# =========================================================
@app.post("/api/map/upload-pcd")
async def upload_pcd(file: UploadFile = File(...)):
    suffix = os.path.splitext(file.filename)[1].lower()
    allow = {".pcd", ".ply", ".xyz", ".xyzn", ".xyzrgb"}
    if suffix not in allow:
        return JSONResponse({"ok": False, "message": f"仅支持 {sorted(list(allow))} 文件"})

    content = await file.read()
    tmp_path = None
    try:
        with tempfile.NamedTemporaryFile(delete=False, suffix=suffix) as tmp:
            tmp.write(content)
            tmp_path = tmp.name

        core.load_pcd_file(tmp_path)
        with core.lock:
            core.pcd_path = file.filename

        return JSONResponse({"ok": True, "pcd_file": file.filename})
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"点云读取失败: {e}"})
    finally:
        if tmp_path and os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except Exception:
                pass


@app.post("/api/map/save-workspace")
async def save_map_workspace():
    try:
        full = export_full_state()
        data = {
            "format": "indoor_route_nav_workspace",
            "version": 1,
            "pcd_path": full.get("pcd_path", ""),
            "points_xyz": full.get("points_xyz", []),
            "points_rgb": full.get("points_rgb", []),
            "drivable_areas": full.get("drivable_areas", []),
            "connectors": full.get("connectors", []),
            "map_is_3d": True,
        }
        text = json.dumps(data, ensure_ascii=False, indent=2)
        return Response(
            content=text,
            media_type="application/json",
            headers={"Content-Disposition": "attachment; filename=indoor_map_workspace.json"}
        )
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"地图工程导出失败: {e}"})


@app.post("/api/map/load-workspace")
async def load_map_workspace(file: UploadFile = File(...)):
    try:
        content = await file.read()
        data = json.loads(content.decode("utf-8"))
        if data.get("format") != "indoor_route_nav_workspace":
            return JSONResponse({"ok": False, "message": "不是 indoor_route_nav 地图工程文件"})

        points_xyz = data.get("points_xyz", [])
        if not isinstance(points_xyz, list) or len(points_xyz) == 0:
            return JSONResponse({"ok": False, "message": "地图工程中没有点云数据"})

        points_rgb = data.get("points_rgb", [])
        drivable_areas = data.get("drivable_areas", [])
        connectors = data.get("connectors", [])

        with core.lock:
            normalized_points = []
            for p in points_xyz:
                normalized_points.append(_point_like_to_xyz(p))

            core.pcd_path = data.get("pcd_path", file.filename)
            core.points_xyz = normalized_points
            core.points_xy = [[p["x"], p["y"]] for p in normalized_points]
            core.points_rgb = points_rgb if isinstance(points_rgb, list) else []
            core.drivable_areas = []
            for i, area in enumerate(drivable_areas if isinstance(drivable_areas, list) else []):
                core.drivable_areas.append(core._normalize_drivable_area(area, i))
            core.connectors = []
            for i, connector in enumerate(connectors if isinstance(connectors, list) else []):
                core.connectors.append(core._normalize_connector(connector, i))
            core.route_polyline = []
            core.route_cumlen = []
            core._reset_navigation_state_locked()
            core._bump_map_rev()
            core._bump_route_rev()
            core._bump_area_rev()
            core.set_status(f"已读取地图工程: {file.filename}")

        return {"ok": True, "message": "地图工程读取成功"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"地图工程读取失败: {e}"})


# =========================================================
# 参数
# =========================================================
@app.post("/api/params/control")
async def set_control_params(payload: dict = Body(...)):
    try:
        core.set_control_params(payload)
        ros_runner.sync_controller_config()
        core.set_status("已应用控制参数")
        return {"ok": True}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/params/smoothing")
async def set_smoothing_params(payload: dict = Body(...)):
    try:
        core.set_smoothing_params(payload)
        reapply_route_sampling_if_possible()
        return {"ok": True}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


# =========================================================
# 路线：读取 JSON / NPZ
# =========================================================
@app.post("/api/route/upload-json")
async def route_upload_json(file: UploadFile = File(...)):
    try:
        content = await file.read()
        points = parse_route_json_bytes(content)
        apply_route_points(points)
        return {"ok": True, "message": f"已加载 JSON 路线: {file.filename}", "count": len(points)}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"JSON 路线读取失败: {e}"})


@app.post("/api/route/upload-npz")
async def route_upload_npz(file: UploadFile = File(...)):
    try:
        content = await file.read()
        points = parse_route_npz_bytes(content)
        apply_route_points(points)
        return {"ok": True, "message": f"已加载 NPZ 路线: {file.filename}", "count": len(points)}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"NPZ 路线读取失败: {e}"})


# =========================================================
# 路线：直接设置路线点（手绘路线提交）
# =========================================================
@app.post("/api/route/set-points")
async def route_set_points(payload: dict = Body(...)):
    try:
        points = payload.get("points", [])
        if not isinstance(points, list) or len(points) < 2:
            return JSONResponse({"ok": False, "message": "路线点数不足（至少需要2个点）"})
        apply_route_points(points)
        return {"ok": True, "message": f"已设置手绘路线，共 {len(points)} 点", "count": len(points)}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"设置路线失败: {e}"})


# =========================================================
# 可行驶区域 / 自动规划
# =========================================================
@app.post("/api/drivable/add")
async def drivable_add(payload: dict = Body(...)):
    try:
        core.add_drivable_area(payload)
        with core.lock:
            areas = copy.deepcopy(core.drivable_areas)
            area_revision = core.area_revision
        return {
            "ok": True,
            "message": "已添加可行驶区域",
            "drivable_areas": areas,
            "area_revision": area_revision,
            "area_count": len(areas),
        }
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/drivable/delete-index")
async def drivable_delete_index(payload: dict = Body(...)):
    try:
        core.delete_drivable_area(int(payload["index"]))
        with core.lock:
            areas = copy.deepcopy(core.drivable_areas)
            area_revision = core.area_revision
        return {
            "ok": True,
            "message": "已删除可行驶区域",
            "drivable_areas": areas,
            "area_revision": area_revision,
            "area_count": len(areas),
        }
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/drivable/clear")
async def drivable_clear():
    try:
        core.clear_drivable_areas()
        with core.lock:
            area_revision = core.area_revision
        return {
            "ok": True,
            "message": "已清空可行驶区域",
            "drivable_areas": [],
            "area_revision": area_revision,
            "area_count": 0,
        }
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/connectors/add")
async def connector_add(payload: dict = Body(...)):
    try:
        core.add_connector(payload)
        return {"ok": True, "message": "已添加楼层连接区"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/connectors/delete-index")
async def connector_delete_index(payload: dict = Body(...)):
    try:
        core.delete_connector(int(payload["index"]))
        return {"ok": True, "message": "已删除楼层连接区"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/connectors/clear")
async def connector_clear():
    try:
        core.clear_connectors()
        return {"ok": True, "message": "已清空楼层连接区"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


def execute_drivable_plan_to_goal(payload: dict):
    goal = _point_like_to_xyz(payload.get("goal", {}))
    resolution = float(payload.get("resolution", 0.20))
    robot_radius = float(payload.get("robot_radius", 0.15))
    final_yaw_deg = payload.get("final_yaw_deg", None)

    with core.lock:
        if core.current_pose is None or not core.localized:
            raise RuntimeError("尚未定位，无法自动规划")

        start = dict(core.current_pose)
        areas = [copy.deepcopy(a) for a in core.drivable_areas]
        connectors = [copy.deepcopy(c) for c in getattr(core, "connectors", [])]
        connectors.extend(corridor_areas_as_connectors(areas))

    if not areas:
        raise RuntimeError("请先绘制可行驶区域")

    start, start_cloud_msg = project_pose_to_cloud_surface(start, "起点", radius=0.80, z_window=1.20)
    start, start_area_msg, _start_area_hit = project_pose_to_lower_drivable_area(
        start, areas, "起点", xy_tolerance=max(0.12, robot_radius), z_tolerance=0.45
    )
    goal, goal_area_msg, goal_area_hit = project_pose_to_lower_drivable_area(
        goal, areas, "目标点", xy_tolerance=max(0.12, robot_radius), z_tolerance=0.45
    )
    projection_msgs = [start_cloud_msg, start_area_msg, goal_area_msg]
    if not goal_area_hit:
        goal, goal_cloud_msg = project_pose_to_cloud_surface(goal, "目标点", radius=0.45, z_window=1.20)
        projection_msgs.append(goal_cloud_msg)

    z_min = payload.get("z_min", None)
    z_max = payload.get("z_max", None)
    if z_min is None and z_max is None and abs(float(start.get("z", 0.0)) - float(goal.get("z", start.get("z", 0.0)))) > 0.45:
        points, planner_message = plan_path_with_connectors(
            start=start,
            goal=goal,
            areas=areas,
            connectors=connectors,
            resolution=resolution,
            robot_radius=robot_radius,
        )
        apply_route_points(points)
        if final_yaw_deg is not None:
            with core.lock:
                core.final_target_yaw = float(final_yaw_deg)
            ros_runner.sync_controller_config()
        return {
            "ok": True,
            "message": planner_message + " | " + "；".join(projection_msgs),
            "count": len(points),
            "points": points,
        }

    if z_min is not None and z_max is not None:
        z_min = float(z_min)
        z_max = float(z_max)
        if z_min > z_max:
            z_min, z_max = z_max, z_min
        if not (z_min <= float(start.get("z", 0.0)) <= z_max):
            raise RuntimeError("当前位置不在当前高度层内，请切换到机器人所在楼层后再规划")
        if not (z_min <= float(goal.get("z", 0.0)) <= z_max):
            raise RuntimeError("目标点不在当前高度层内")
        areas = [
            a for a in areas
            if float(a.get("z_min", -1e18)) <= float(goal.get("z", 0.0)) <= float(a.get("z_max", 1e18))
            and float(a.get("z_max", 0.0)) >= z_min
            and float(a.get("z_min", 0.0)) <= z_max
        ]
    else:
        gz = float(goal.get("z", start.get("z", 0.0)))
        areas = [
            a for a in areas
            if float(a.get("z_min", -1e18)) <= gz <= float(a.get("z_max", 1e18))
        ]

    points = plan_path_native(
        start=start,
        goal=goal,
        areas=areas,
        resolution=resolution,
        robot_radius=robot_radius,
    )
    planner_message = f"C++自动规划完成，路径点 {len(points)} 个"
    apply_route_points(points)
    if final_yaw_deg is not None:
        with core.lock:
            core.final_target_yaw = float(final_yaw_deg)
        ros_runner.sync_controller_config()
    return {
        "ok": True,
        "message": planner_message + " | " + "；".join(projection_msgs),
        "count": len(points),
        "points": points,
    }


@app.post("/api/drivable/plan-to-goal")
async def drivable_plan_to_goal(payload: dict = Body(...)):
    try:
        return execute_drivable_plan_to_goal(payload)
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"自动规划失败: {e}"})


# =========================================================
# 路线：保存路线到文件
# =========================================================
@app.post("/api/route/save-to-path")
async def route_save_to_path(payload: dict = Body(...)):
    try:
        file_path = payload.get("file_path", "route.json")
        ensure_parent_dir(file_path)
        
        if hasattr(core, "export_state"):
            state = core.export_state()
        else:
            state = {}
        
        points = state.get("route_polyline", [])
        if not points or len(points) < 2:
            return JSONResponse({"ok": False, "message": "路线点数不足，无法保存"})
        
        # 转换为列表格式
        export_data = {
            "route_points": [
                {
                    "x": float(p.get("x", 0)),
                    "y": float(p.get("y", 0)),
                    "z": float(p.get("z", 0))
                } for p in points
            ]
        }
        
        with open(file_path, "w") as f:
            json.dump(export_data, f, indent=2)
        
        return {"ok": True, "message": f"已保存路线到: {file_path}"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"保存失败: {e}"})


# =========================================================
# 系统参数
# =========================================================
@app.post("/api/config/save-system-params")
async def config_save_system_params():
    try:
        yml = core.dump_system_params_yaml()
        return Response(
            content=yml,
            media_type="application/x-yaml",
            headers={"Content-Disposition": "attachment; filename=system_params.yaml"}
        )
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"系统参数导出失败: {e}"})


@app.post("/api/config/load-system-params")
async def config_load_system_params(file: UploadFile = File(...)):
    try:
        content = await file.read()
        data = yaml.safe_load(content.decode("utf-8")) or {}
        core.load_system_params_data(data)
        return {"ok": True}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"系统参数配置读取失败: {e}"})


@app.post("/api/config/save-system-params-to-path")
async def config_save_system_params_to_path(payload: dict = Body(...)):
    try:
        file_path = str(payload.get("file_path", "")).strip()
        if not file_path:
            return JSONResponse({"ok": False, "message": "file_path 不能为空"})

        if not (file_path.endswith(".yaml") or file_path.endswith(".yml")):
            return JSONResponse({"ok": False, "message": "文件名必须以 .yaml 或 .yml 结尾"})

        yml = core.dump_system_params_yaml()
        ensure_parent_dir(file_path)
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(yml)

        return {"ok": True, "message": f"已保存参数到: {file_path}"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"保存失败: {e}"})


# =========================================================
# 导航
# =========================================================
@app.post("/api/nav/start")
async def nav_start():
    try:
        ros_runner.start_navigation()
        return {"ok": True, "message": "导航已启动"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/nav/stop")
async def nav_stop():
    try:
        ros_runner.stop_navigation()
        return {"ok": True, "message": "导航已停止"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/nav/emergency")
async def nav_emergency():
    try:
        ros_runner.emergency_stop_navigation()
        return {"ok": True, "message": "已紧急停止"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/nav/clear")
async def nav_clear():
    try:
        ros_runner.clear_navigation()
        return {"ok": True, "message": "已清空当前路线"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


@app.post("/api/ros/initialpose")
async def ros_initialpose(payload: dict = Body(...)):
    try:
        x = float(payload.get("x", 0.0))
        y = float(payload.get("y", 0.0))
        z = float(payload.get("z", 0.0))
        yaw_deg = float(payload.get("yaw_deg", 0.0))
        ros_runner.publish_initial_pose(x, y, z, yaw_deg)
        return {"ok": True, "message": f"已发布 /initialpose"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": str(e)})


# 兼容旧接口
@app.post("/api/route/clear")
async def route_clear_compat():
    return await nav_clear()


# =========================================================
# 底盘控制
# =========================================================
@app.post("/api/chassis/cmd_vel")
async def chassis_cmd_vel(payload: dict = Body(...)):
    try:
        vx = float(payload.get("vx", 0.0))
        vy = float(payload.get("vy", 0.0))
        vz = float(payload.get("vz", 0.0))
        ros_runner.chassis_cmd_vel(vx, vy, vz)
        return {"ok": True, "message": f"cmd_vel: vx={vx:.2f} vy={vy:.2f} wz={vz:.2f}"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"cmd_vel FAIL: {e}"})


@app.post("/api/chassis/mode")
async def chassis_mode(payload: dict = Body(...)):
    try:
        mode = str(payload.get("mode", "stand"))
        enable = bool(payload.get("enable", True))
        ros_runner.chassis_mode(mode, enable)
        return {"ok": True, "message": f"chassis mode: {mode} enable={enable}"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"mode FAIL: {e}"})


@app.post("/api/chassis/sitdown")
async def chassis_sitdown():
    try:
        ros_runner.chassis_sitdown()
        return {"ok": True, "message": "sitdown 已发送"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"sitdown FAIL: {e}"})


@app.post("/api/chassis/emgy_stop")
async def chassis_emgy_stop():
    try:
        ros_runner.chassis_emgy_stop()
        return {"ok": True, "message": "紧急停止已发送"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"emgy_stop FAIL: {e}"})


@app.post("/api/chassis/imu_enable")
async def chassis_imu_enable(payload: dict = Body(...)):
    try:
        enable = bool(payload.get("enable", True))
        ros_runner.chassis_imu_enable(enable)
        return {"ok": True, "message": f"IMU enable={enable}"}
    except Exception as e:
        return JSONResponse({"ok": False, "message": f"imu_enable FAIL: {e}"})


# =========================================================
# ROS 图像 MJPEG
# =========================================================
def mjpeg_generator():
    last_sent_time = 0.0
    while True:
        try:
            if hasattr(core, "get_camera_frame"):
                frame, ts = core.get_camera_frame()
            else:
                frame, ts = None, 0.0
        except Exception:
            frame, ts = None, 0.0

        if frame is not None and ts != last_sent_time:
            last_sent_time = ts
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" +
                frame +
                b"\r\n"
            )
        else:
            time.sleep(0.03)


@app.get("/api/camera/mjpeg")
async def camera_mjpeg():
    return StreamingResponse(
        mjpeg_generator(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


# =========================================================
# 系统
# =========================================================
def delayed_shutdown():
    time.sleep(0.8)

    try:
        core.emergency_stop()
    except Exception:
        pass

    try:
        ros_runner.stop()
    except Exception:
        pass

    try:
        os.kill(os.getpid(), signal.SIGTERM)
    except Exception:
        pass

    time.sleep(0.5)
    os._exit(0)


@app.post("/api/system/shutdown")
async def system_shutdown():
    threading.Thread(target=delayed_shutdown, daemon=True).start()
    return {"ok": True, "message": "程序即将关闭"}


# =========================================================
# WebSocket：只发轻量状态
# =========================================================
@app.websocket("/ws/status")
async def ws_status(websocket: WebSocket):
    await websocket.accept()
    ws_send_seq = 0
    import sys
    try:
        while True:
            ws_send_seq += 1
            data = build_compact_state()
            if ws_send_seq % 50 == 1:
                cp = data.get("current_pose") if isinstance(data, dict) else None
                clusters = data.get("obstacle_clusters", []) if isinstance(data, dict) else []
                obs_dist = data.get("obstacle_distance", 999) if isinstance(data, dict) else 999
                if cp:
                    print(f"[WS send #{ws_send_seq}] pose: x={cp.get('x',0):.2f} y={cp.get('y',0):.2f} "
                          f"obstacles={len(clusters)} dist={obs_dist:.1f}", flush=True)
                else:
                    print(f"[WS send #{ws_send_seq}] pose=NULL obstacles={len(clusters)}", flush=True)
            await websocket.send_text(json.dumps(data, ensure_ascii=False))
            await asyncio.sleep(0.2)
    except WebSocketDisconnect:
        print("[WS] client disconnected", flush=True)
        return
    except Exception as e:
        print(f"[WS] error: {e}", flush=True)
        return


# =========================================================
# 启动辅助
# =========================================================
def show_startup_native_popup(local_url: str, lan_url: str):
    text = f"本机访问:  {local_url}\n局域网访问: {lan_url}"

    try:
        if shutil.which("zenity"):
            subprocess.Popen([
                "zenity",
                "--info",
                "--title=程序启动成功",
                "--width=520",
                "--height=220",
                f"--text={text}"
            ])
            return
    except Exception:
        pass

    try:
        if shutil.which("xmessage"):
            subprocess.Popen([
                "xmessage",
                "-center",
                text
            ])
            return
    except Exception:
        pass

    try:
        def _popup():
            import tkinter as tk
            root = tk.Tk()
            root.title("程序启动成功")
            root.geometry("520x180")
            root.resizable(False, False)

            try:
                root.attributes("-topmost", True)
            except Exception:
                pass

            frame = tk.Frame(root, padx=16, pady=16)
            frame.pack(fill="both", expand=True)

            title = tk.Label(frame, text="程序已启动，可通过以下地址访问：", font=("Arial", 12, "bold"))
            title.pack(anchor="w", pady=(0, 12))

            local_var = tk.StringVar(value=local_url)
            lan_var = tk.StringVar(value=lan_url)

            tk.Label(frame, text="本机访问:", anchor="w").pack(fill="x")
            e1 = tk.Entry(frame, textvariable=local_var, font=("Arial", 10))
            e1.pack(fill="x", pady=(2, 8))

            tk.Label(frame, text="局域网访问:", anchor="w").pack(fill="x")
            e2 = tk.Entry(frame, textvariable=lan_var, font=("Arial", 10))
            e2.pack(fill="x", pady=(2, 12))

            btn_frame = tk.Frame(frame)
            btn_frame.pack(fill="x")

            def copy_local():
                root.clipboard_clear()
                root.clipboard_append(local_url)
                root.update()

            def copy_lan():
                root.clipboard_clear()
                root.clipboard_append(lan_url)
                root.update()

            tk.Button(btn_frame, text="复制本机地址", command=copy_local).pack(side="left", padx=(0, 8))
            tk.Button(btn_frame, text="复制局域网地址", command=copy_lan).pack(side="left", padx=(0, 8))
            tk.Button(btn_frame, text="关闭", command=root.destroy).pack(side="right")

            root.mainloop()

        threading.Thread(target=_popup, daemon=True).start()
    except Exception:
        pass


def get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


# =========================================================
# 启动
# =========================================================
if __name__ == "__main__":
    ip = get_local_ip()
    local_url = f"http://127.0.0.1:{APP_PORT}"
    lan_url = f"http://{ip}:{APP_PORT}"

    topics = build_topics_dict()

    print("=" * 60)
    print(f"本机访问:  {local_url}")
    print(f"局域网访问: {lan_url}")
    print("=" * 60)
    print(f"图像话题: {topics.get('camera_topic', '-')}")
    print(f"开启导航话题: {topics.get('nav_start_topic', '-')}")
    print(f"停止导航话题: {topics.get('nav_stop_topic', '-')}")
    print(f"导航完成话题: {topics.get('nav_done_topic', '-')}")
    print(f"外部目标点话题: {topics.get('external_goal_topic', '-')}")
    print(f"initialpose frame: {topics.get('initialpose_frame', 'map')}")

    show_startup_native_popup(local_url, lan_url)

    uvicorn.run(app, host="0.0.0.0", port=APP_PORT, reload=False)