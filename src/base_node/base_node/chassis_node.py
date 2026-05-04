#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import sys
import time
import uuid
import threading
from functools import partial

import websocket
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist, Quaternion, Vector3
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Imu

try:
    import limxsdk.robot.Robot as Robot
    import limxsdk.robot.RobotType as RobotType
    import limxsdk.datatypes as datatypes
    HAS_LIMXSDK = True
except ImportError:
    HAS_LIMXSDK = False


def generate_guid():
    return str(uuid.uuid4())


class UnifiedChassisNode(Node):

    def __init__(self):
        super().__init__("unified_chassis_node")

        self.declare_parameter("robot_ip", "10.192.1.2")
        self.declare_parameter("ws_port", 5000)
        self.declare_parameter("cmd_vel_rate", 100.0)
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("max_linear_x", 2.0)
        self.declare_parameter("max_linear_y", 2.0)
        self.declare_parameter("max_angular_z", 2.0)

        self.robot_ip = self.get_parameter("robot_ip").value
        self.ws_port = self.get_parameter("ws_port").value
        self.cmd_vel_rate = self.get_parameter("cmd_vel_rate").value
        self.imu_frame_id = self.get_parameter("imu_frame_id").value
        self.max_vx = self.get_parameter("max_linear_x").value
        self.max_vy = self.get_parameter("max_linear_y").value
        self.max_wz = self.get_parameter("max_angular_z").value

        self.accid = None
        self.ws_connected = False
        self.ws_client = None
        self.should_exit = False

        self.last_twist_time = 0.0
        self.twist_interval = 1.0 / max(1.0, self.cmd_vel_rate)

        self._cmd_seq = 0
        self._imu_seq = 0

        # ========== Subscriptions (commands IN) ==========
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        self.mode_sub = self.create_subscription(
            String, "/chassis/mode", self.mode_callback, 10
        )

        self.sitdown_sub = self.create_subscription(
            Empty, "/chassis/sitdown", self.sitdown_callback, 10
        )

        self.emgy_stop_sub = self.create_subscription(
            Empty, "/chassis/emgy_stop", self.emgy_stop_callback, 10
        )

        self.imu_enable_sub = self.create_subscription(
            String, "/chassis/imu_enable", self.imu_enable_callback, 10
        )

        self.raw_cmd_sub = self.create_subscription(
            String, "/chassis/raw_command", self.raw_command_callback, 10
        )

        # ========== Publishers (data OUT) ==========
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
        )
        self.imu_pub = self.create_publisher(Imu, "/imu", imu_qos)

        self.state_pub = self.create_publisher(String, "/chassis/state", 10)
        self.mode_response_pub = self.create_publisher(
            String, "/chassis/mode_response", 10
        )

        # ========== Status timer ==========
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f"统一底盘节点启动, 目标机器人: {self.robot_ip}:{self.ws_port}"
        )
        self.get_logger().info(
            "支持指令: twist | stand | walk | sitdown | stair | emgy_stop | imu_enable"
        )

        # ========== Start connections ==========
        self._start_ws()
        if HAS_LIMXSDK:
            self._start_limxsdk()
        else:
            self.get_logger().warn("limxsdk 未安装，IMU 数据不可用")

    # =========================================================
    # WebSocket
    # =========================================================
    def _start_ws(self):
        ws_url = f"ws://{self.robot_ip}:{self.ws_port}"
        self.ws_client = websocket.WebSocketApp(
            ws_url,
            on_open=self._ws_on_open,
            on_message=self._ws_on_message,
            on_close=self._ws_on_close,
            on_error=self._ws_on_error,
        )
        ws_thread = threading.Thread(
            target=self.ws_client.run_forever,
            kwargs={"ping_interval": 10, "ping_timeout": 5},
            daemon=True,
        )
        ws_thread.start()
        self.get_logger().info(f"WebSocket 连接中: {ws_url}")

    def _ws_on_open(self, ws):
        self.ws_connected = True
        self.get_logger().info("WebSocket 已连接, 等待 ACCID ...")
        self._publish_state("connected", {"message": "WebSocket connected"})

    def _ws_on_message(self, ws, message):
        try:
            root = json.loads(message)
        except json.JSONDecodeError:
            return

        self.accid = root.get("accid", self.accid)
        title = root.get("title", "")
        data = root.get("data", {})

        mode_response_titles = {
            "response_stand_mode",
            "response_walk_mode",
            "response_stair_mode",
            "response_sitdown",
        }

        if title == "notify_robot_info":
            if self.accid:
                self.get_logger().info(f"ACCID: {self.accid}")
            self._publish_state("robot_info", {"data": data})

        elif title in mode_response_titles:
            self.mode_response_pub.publish(String(data=message))
            self._publish_state(title, {"data": data})

        elif title == "response_emgy_stop":
            self._publish_state("emgy_stop_response", {"data": data})

        elif title == "response_enable_imu":
            self._publish_state("imu_enable_response", {"data": data})

        else:
            self._publish_state(title, {"data": data} if data else {})

    def _ws_on_close(self, ws, code, msg):
        self.ws_connected = False
        self.get_logger().warn(f"WebSocket 断开 (code={code}), 将自动重连")
        self._publish_state("disconnected", {"code": code, "msg": str(msg)})

    def _ws_on_error(self, ws, error):
        self.get_logger().error(f"WebSocket 错误: {error}")

    def _ws_send(self, title, data=None):
        if data is None:
            data = {}
        if not self.ws_connected or not self.ws_client:
            return False
        if self.accid is None and title not in ("",):
            return False
        try:
            msg = {
                "accid": self.accid if self.accid else "",
                "title": title,
                "timestamp": int(time.time() * 1000),
                "guid": generate_guid(),
                "data": data,
            }
            self.ws_client.send(json.dumps(msg))
            return True
        except Exception as e:
            self.get_logger().error(f"WS 发送失败 [{title}]: {e}")
            return False

    # =========================================================
    # limxsdk (sensors)
    # =========================================================
    def _start_limxsdk(self):
        try:
            self.limx_robot = Robot(RobotType.PointFoot)
            if not self.limx_robot.init(self.robot_ip):
                self.get_logger().error(f"limxsdk 初始化失败, IP: {self.robot_ip}")
                return

            imu_cb = partial(self._imu_callback)
            self.limx_robot.subscribeImuData(imu_cb)
            self.get_logger().info("limxsdk IMU 订阅已启动")
        except Exception as e:
            self.get_logger().error(f"limxsdk 启动失败: {e}")

    def _imu_callback(self, imu_data: datatypes.ImuData):
        try:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.imu_frame_id
            msg.linear_acceleration = Vector3(
                x=float(imu_data.acc[0]),
                y=float(imu_data.acc[1]),
                z=float(imu_data.acc[2]),
            )
            msg.angular_velocity = Vector3(
                x=float(imu_data.gyro[0]),
                y=float(imu_data.gyro[1]),
                z=float(imu_data.gyro[2]),
            )
            msg.orientation = Quaternion(
                x=float(imu_data.quat[0]),
                y=float(imu_data.quat[1]),
                z=float(imu_data.quat[2]),
                w=float(imu_data.quat[3]),
            )
            self.imu_pub.publish(msg)
            self._imu_seq += 1
        except Exception as e:
            self.get_logger().error(f"IMU 发布失败: {e}")

    # =========================================================
    # ROS2 Callbacks — 全部 7 个 SDK 指令
    # =========================================================

    # 1. request_twist
    def cmd_vel_callback(self, msg: Twist):
        now = time.time()
        if now - self.last_twist_time < self.twist_interval:
            return
        self.last_twist_time = now

        x = max(min(float(msg.linear.x), self.max_vx), -self.max_vx)
        y = max(min(float(msg.linear.y), self.max_vy), -self.max_vy)
        z = max(min(float(msg.angular.z), self.max_wz), -self.max_wz)

        self._ws_send("request_twist", {"x": x, "y": y, "z": z})

        self._cmd_seq += 1
        if self._cmd_seq % 100 == 1:
            self.get_logger().info(
                f"twist #{self._cmd_seq}: vx={x:.2f} vy={y:.2f} wz={z:.2f}"
            )

    # 2. request_stand_mode  /  3. request_walk_mode
    # 4. request_stair_mode  /  5. request_sitdown
    def mode_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"无法解析 mode 消息: {msg.data}")
            return

        mode = payload.get("mode", "")
        enable = payload.get("enable", True)

        title_map = {
            "stand": "request_stand_mode",
            "walk": "request_walk_mode",
            "stair": "request_stair_mode",
            "sit": "request_sitdown",
        }

        title = title_map.get(mode)
        if title is None:
            self.get_logger().error(f"未知模式: {mode}, 支持: stand/walk/stair/sit")
            return

        ok = self._ws_send(title, {"enable": enable} if mode == "stair" else None)
        self.get_logger().info(
            f"模式切换: {mode} enable={enable} {'OK' if ok else 'FAIL'}"
        )

    # 5. request_sitdown (also mapped via /chassis/sitdown for direct trigger)
    def sitdown_callback(self, _msg: Empty):
        ok = self._ws_send("request_sitdown")
        self.get_logger().info(f"Sitdown {'OK' if ok else 'FAIL'}")

    # 6. request_emgy_stop
    def emgy_stop_callback(self, _msg: Empty):
        ok = self._ws_send("request_emgy_stop")
        self.get_logger().info(f"Emergency stop {'OK' if ok else 'FAIL'}")

    # 7. request_enable_imu
    def imu_enable_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"无法解析 imu_enable 消息: {msg.data}")
            return

        enable = payload.get("enable", True)
        ok = self._ws_send("request_enable_imu", {"enable": enable})
        self.get_logger().info(
            f"IMU enable={enable} {'OK' if ok else 'FAIL'}"
        )

    # 通用透传指令
    def raw_command_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            title = payload.get("title", "")
            data = payload.get("data", {})
            if title:
                self._ws_send(title, data)
                self.get_logger().info(f"Raw 指令已发送: {title}")
        except json.JSONDecodeError:
            self.get_logger().error(f"无法解析 raw_command: {msg.data}")

    # =========================================================
    # Status
    # =========================================================
    def _publish_state(self, event, extra=None):
        if extra is None:
            extra = {}
        state = {
            "event": event,
            "timestamp": time.time(),
            "ws_connected": self.ws_connected,
            "has_accid": self.accid is not None,
        }
        state.update(extra)
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def publish_status(self):
        state = {
            "event": "heartbeat",
            "timestamp": time.time(),
            "ws_connected": self.ws_connected,
            "has_accid": self.accid is not None,
            "imu_seq": self._imu_seq,
            "cmd_seq": self._cmd_seq,
        }
        self.state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    # =========================================================
    # Shutdown
    # =========================================================
    def shutdown(self):
        self.should_exit = True
        if self.ws_client:
            try:
                self.ws_client.close()
            except Exception:
                pass
        if HAS_LIMXSDK and hasattr(self, "limx_robot"):
            try:
                self.limx_robot.disconnect()
            except Exception:
                pass
        self.get_logger().info("统一底盘节点已关闭")

    def destroy_node(self):
        self.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = UnifiedChassisNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号")
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
