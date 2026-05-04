# ROS2 Indoor Route Navigation


基于 ROS2 Humble 的室内路线导航系统，集成 3D 点云可视化、可行驶区域编辑、A* 自动路径规划与 Pure Pursuit 轨迹跟踪。专为 D-Robotics RDK S100 嵌入式平台优化。

---

## 功能特性

- **3D 点云可视化** — 加载 PCD / PLY / XYZ 格式点云，3D 自由视角旋转缩放
- **可行驶区域编辑** — 多边形绘制、中心线走廊生成，支持多楼层分层管理
- **A* 自动规划** — C++ 实现 8 邻域 A* 搜索 + 路径拉直平滑，含障碍物避让
- **Pure Pursuit 控制** — 前视距离自适应 + 限速曲率调整，输出 `/cmd_vel`
- **Web 操作面板** — FastAPI + WebSocket 实时推送，浏览器内完成全部操作
- **地图工程持久化** — 保存/加载 JSON 工程文件，含采样点云和可行驶区域
- **外部路线接入** — 支持 YAML / JSON 路线包下发，自动启动导航

## 硬件平台

| 项目 | 规格 |
|------|------|
| 主控 | D-Robotics RDK S100 |
| CPU | ARM Cortex-A78AE × 6 |
| 内存 | 4.7 GB |
| 系统 | Ubuntu 22.04 + Linux 6.1 PREEMPT_RT |
| ROS | TROS (TogetheROS.Bot) / ROS2 Humble |

## 目录结构

```
ros2-indoor-navigation/
└── src/
    ├── indoor_route_nav/          # 核心导航包 (ament_cmake)
    │   ├── CMakeLists.txt         # C++ 编译 + Python 安装
    │   ├── package.xml
    │   ├── launch/
    │   │   └── indoor_route_nav.launch.py
    │   ├── config/
    │   │   ├── default_params.yaml
    │   │   └── map_preprocessor.yaml
    │   ├── src/                   # C++ 节点
    │   │   ├── indoor_route_planner_node.cpp
    │   │   ├── indoor_route_controller_node.cpp
    │   │   ├── indoor_map_preprocessor_node.cpp
    │   │   └── indoor_planning_bindings.cpp
    │   ├── scripts/               # Python 节点
    │   │   ├── web_app.py         # Web 服务入口
    │   │   ├── core_ros.py        # ROS 桥接核心
    │   │   └── publish_pcl_pose.py
    │   ├── srv/PlanPath.srv       # 规划服务接口
    │   ├── static/                # Web 前端
    │   │   ├── index.html         # 3D 面板 (Canvas)
    │   │   └── mouse_coord.js
    │   └── requirements.txt
    └── base_node/                 # 底盘控制包 (ament_python)
        ├── setup.py
        ├── package.xml
        └── base_node/
            └── chassis_node.py    # 底盘驱动 + IMU 发布
```

## 系统架构

```
                    ┌──────────────────────────────┐
                    │         Web Browser           │
                    │  Canvas 3D + Control Panel    │
                    └──────────▲──┬─────────────────┘
                    HTTP/WS     │  │
                    ┌───────────┴──▼─────────────────┐
                    │        web_app.py              │
                    │  FastAPI + WebSocket Bridge    │
                    └───────────▲──┬─────────────────┘
                    ROS2 Topic  │  │ ROS2 Service
           ┌────────────────────┼──┼──────────────────┐
           │                    │  │                   │
           ▼                    ▼  ▼                   ▼
  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────────┐
  │ route_planner   │  │  route_controller│  │ map_preprocessor │
  │   (C++, A*)     │  │  (C++, Pursuit)  │  │   (C++, PCL)     │
  └────────┬────────┘  └────────┬────────┘  └──────────────────┘
           │                    │
           ▼                    ▼
      /plan_path           /cmd_vel
      nav_msgs/Path     geometry_msgs/Twist
                                 │
                                 ▼
                        ┌─────────────────┐
                        │    base_node    │
                        │  chassis + IMU  │
                        └─────────────────┘
```

## 快速开始

### 前置依赖

- ROS2 Humble (或 TROS)
- Python 3.10+ (FastAPI, uvicorn, numpy, opencv, pyyaml)
- PCL 1.12+ (点云处理)
- yaml-cpp (配置解析)
- pybind11 (C++/Python 绑定)

### 构建

```bash
cd ~/robot_ws
colcon build --packages-select base_node indoor_route_nav
source install/setup.bash
```

### 启动完整系统

```bash
ros2 launch indoor_route_nav indoor_route_nav.launch.py
```

仅启动 Web 桥接（不使用 C++ 规划/控制）：

```bash
ros2 launch indoor_route_nav indoor_route_nav.launch.py \
  use_cpp_planner:=false use_cpp_controller:=false
```

### 访问界面

浏览器打开 `http://<设备IP>:8000`

### 测试定位

```bash
ros2 run indoor_route_nav publish_pcl_pose --x 0.0 --y 0.0 --z 0.0 --yaw 0 --rate 5
```

## C++ 节点

### indoor_route_planner_node

| 接口 | 类型 | 说明 |
|------|------|------|
| Service `/indoor_route_nav/planner/plan_path` | `PlanPath.srv` | 规划请求 |
| Topic `/indoor_route_nav/planner/path` | `nav_msgs/Path` | 规划结果 |
| Topic `/indoor_route_nav/planner/status` | `std_msgs/String` | 状态文本 |

算法：
- 8 邻域 A* 网格搜索
- 可行驶区域栅格化 + 边界半径约束
- 路径拉直 (line-of-sight pruning)
- 三次样条平滑
- 坡道 / 楼梯 Z 值自动估算

### indoor_route_controller_node

| 接口 | 类型 | 说明 |
|------|------|------|
| Topic `/indoor_route_nav/controller/route` | `nav_msgs/Path` | 目标路线 |
| Topic `/indoor_route_nav/controller/config` | `std_msgs/String` | 控制参数 JSON |
| Service `/indoor_route_nav/controller/start` | `std_srvs/Trigger` | 开始跟踪 |
| Service `/indoor_route_nav/controller/stop` | `std_srvs/Trigger` | 停止 |
| Topic `/cmd_vel` | `geometry_msgs/Twist` | 速度指令 |

控制策略：
- Pure Pursuit 前视距离自适应
- 曲率限速
- 障碍物紧急制动
- 路点精准停靠 (stop on arrival)
- 空间网格索引加速近邻查找

### indoor_map_preprocessor_node

```bash
ros2 run indoor_route_nav indoor_map_preprocessor_node --config \
  config/map_preprocessor.yaml
```

- 输入：PCD / PLY / XYZ 点云文件
- 输出：Web 可加载的 JSON 工程文件
- 参数：体素降采样、Z 轴裁剪、格式转换

## ROS2 话题总览

### 订阅

| 话题 | 类型 | 用途 |
|------|------|------|
| `/pcl_pose` | `PoseWithCovarianceStamped` | 机器人定位 |
| `/front_obstacle/avg_distance` | `Float32` | 前方障碍距离 |
| `/external_route_path` | `nav_msgs/Path` | 外部路线 |
| `/external_route_rich` | `String` | 路线包 (JSON) |
| `/external_stop_points` | `String` | 路点 JSON |
| `/nav/start` | `Empty` | 启动导航 |
| `/nav/clear` | `Empty` | 清除路线 |
| `/indoor_route_nav/goal` | `PoseStamped` | 单点导航目标 |
| `/camera/image_raw` | `Image` | 实时图像流 |

### 发布

| 话题 | 类型 | 用途 |
|------|------|------|
| `/cmd_vel` | `Twist` | 速度控制指令 |
| `/initialpose` | `PoseWithCovarianceStamped` | 初始位姿 |
| `/nav/done` | `String` | 导航完成事件 |

## 路线包格式

### JSON

```json
{
  "points": [
    {"x": 0.0, "y": 0.0, "z": 0.0},
    {"x": 1.0, "y": 0.0, "z": 0.0}
  ],
  "stop_points": [
    {"name": "P0", "x": 1.0, "y": 0.0, "z": 0.0, "yaw_deg": 90.0, "stop_time": 2.0}
  ],
  "speed_params": {"max_linear_x": 0.5, "max_angular_z": 0.4},
  "auto_start": true
}
```

### YAML

```yaml
route:
  polyline:
    - {x: 0.0, y: 0.0, z: 0.0}
    - {x: 1.0, y: 0.0, z: 0.0}
stop_points:
  - {name: P0, x: 1.0, y: 0.0, z: 0.0, yaw_deg: 90.0, stop_time: 2.0}
```

## 性能优化

针对 RDK S100 ARM 平台的优化措施：

| 优化项 | 方案 | 效果 |
|--------|------|------|
| 编译器 | `-O3 -march=native -flto` | ~15-25% 加速 |
| 近邻查找 | 空间网格索引 | O(N×M) → O(N) |
| 状态发布 | 20 Hz → 5 Hz 节流 | 减少序列化开销 |
| 日志 | INFO → DEBUG | 消除 I/O 瓶颈 |
| 障碍数据 | 关闭避障时跳过处理 | 减少无效计算 |
| Web 渲染 | requestAnimationFrame | 浏览器端帧合并 |
| 前端缓存 | map points / sin-cos / DOM | 减少重计算 |

## License

This project is proprietary. All rights reserved.
