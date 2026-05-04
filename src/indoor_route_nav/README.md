# Indoor Route Nav

这是一个基于 FastAPI 的 ROS2 室内路线导航控制界面，用于加载 3D 点云地图、绘制或导入路线、配置路点，并通过 ROS2 发布 `/cmd_vel` 控制机器人沿路线行驶。

ROS2 包名：`indoor_route_nav`

推荐工作空间名：`indoor_nav_ws`

## 启动

先进入 ROS2 环境，再启动 Web 服务：

```bash
cd /home/if/indoor_nav_ws/src/indoor_route_nav
python3 web_app.py
```

也可以作为 ROS2 功能包构建后启动：

```bash
cd /home/if/indoor_nav_ws
colcon build --packages-select indoor_route_nav
source install/setup.bash
ros2 launch indoor_route_nav indoor_route_nav.launch.py
```

只启动 Python Web/ROS 桥，不启动 C++ 规划节点：

```bash
ros2 launch indoor_route_nav indoor_route_nav.launch.py use_cpp_planner:=false
```

也可以分别启动：

```bash
ros2 run indoor_route_nav web_app
ros2 run indoor_route_nav indoor_route_planner_node
```

测试发布当前位置：

```bash
ros2 run indoor_route_nav publish_pcl_pose --x 0.0 --y 0.0 --z 0.0 --yaw 0.0
ros2 run indoor_route_nav publish_pcl_pose --x 1.2 --y 3.4 --z 0.0 --yaw 90 --rate 5
```

默认访问地址：

- 本机：`http://127.0.0.1:8000`
- 局域网：程序启动时会在终端打印实际 IP 地址

## Python 依赖

通用 Python 依赖见 `requirements.txt`。ROS2 相关依赖需要来自 ROS 环境，不建议用 pip 安装：

- `rclpy`
- `geometry_msgs`
- `nav_msgs`
- `sensor_msgs`
- `std_msgs`
- `cv_bridge`

## C++ 扩展点

这个工程现在是 `ament_cmake` ROS2 功能包，可以同时安装 Python Web 程序并编译 C++ 节点。

当前 C++ 规划节点：

- 节点：`indoor_route_planner_node`
- 服务：`/indoor_route_nav/planner/plan_path`，类型是 `indoor_route_nav/srv/PlanPath`
- 发布：`/indoor_route_nav/planner/path`，类型是 `nav_msgs/Path`
- 状态：`/indoor_route_nav/planner/status`，类型是 `std_msgs/String`

当前实现已经把可行驶区域栅格化、边界半径检查、8 邻域 A*、路径拉直和平滑、坡道/楼梯 Z 值估计放到 C++。Python Web 后端会优先调用这个服务；如果服务没启动或接口类型不可用，会回退到 Python 规划。

当前 C++ 控制节点：

- 节点：`indoor_route_controller_node`
- 输入路线：`/indoor_route_nav/controller/route`，类型是 `nav_msgs/Path`
- 输入参数：`/indoor_route_nav/controller/config`，类型是 `std_msgs/String`
- 控制命令：`/indoor_route_nav/controller/start`、`/indoor_route_nav/controller/stop`、`/indoor_route_nav/controller/clear`
- 输出速度：`/cmd_vel`，类型是 `geometry_msgs/Twist`
- 输出状态：`/indoor_route_nav/controller/state`，类型是 `std_msgs/String`

Web 后端不再直接发布 `/cmd_vel`；它只负责把路线、参数和启动/停止命令同步给 C++ 控制器。

当前 C++ 地图预处理节点：

- 节点：`indoor_map_preprocessor_node`
- 默认配置：`config/map_preprocessor.yaml`
- 输入：PCD / PLY / XYZ 点云文件
- 输出：Web 可直接读取的 `indoor_route_nav_workspace` JSON 文件

示例：

```bash
ros2 run indoor_route_nav indoor_map_preprocessor_node --config \
  /home/if/indoor_nav_ws/src/indoor_route_nav/config/map_preprocessor.yaml
```

配置文件里的 `input_path` 和 `output_path` 分别指定原始点云和输出工程文件。生成的 `preprocessed_map.json` 可以在网页里通过“读取地图工程”加载。

网页端不再提供整图自动标定楼梯入口。楼梯、坡道和走廊推荐使用“中心线生成区域”：先沿点云中心线点几个点，再按设定宽度扩张为可行驶区域。

后续更适合继续放到 C++ 的内容：

- Theta* / Hybrid A*
- 多楼层连接图搜索
- 大点云空间索引和近邻查询
- 更完整的 Pure Pursuit / Stanley / TEB / MPC 控制器

## ROS2 话题

订阅：

- `/pcl_pose`：`geometry_msgs/PoseWithCovarianceStamped`，当前定位
- `/front_obstacle/avg_distance`：`std_msgs/Float32`，前方障碍距离
- `/external_route_path`：`nav_msgs/Path`，外部路线
- `/external_route_rich`：`std_msgs/String`，路线包，兼容 `send.py`
- `/external_stop_points`：`std_msgs/String`，外部路点 JSON
- `/nav/start`：`std_msgs/Empty`，开始导航
- `/nav/clear`：`std_msgs/Empty`，清空路线和路点
- `/indoor_route_nav/goal`：`geometry_msgs/PoseStamped`，外部目标点。收到后会用当前可行驶区域自动规划到该点，姿态 yaw 作为终点朝向。
- `/camera/image_raw`：`sensor_msgs/Image`，Web 图像流

发布：

- `/cmd_vel`：`geometry_msgs/Twist`，速度控制
- `/initialpose`：`geometry_msgs/PoseWithCovarianceStamped`，初始位姿
- `/nav/done`：`std_msgs/String`，导航完成事件

## 路线包格式

`send.py` 默认发布到 `/external_route_rich`，消息内容是 JSON 字符串：

```json
{
  "points": [
    {"x": 0.0, "y": 0.0, "z": 0.0},
    {"x": 1.0, "y": 0.0, "z": 0.0}
  ],
  "stop_points": [
    {"name": "P0", "x": 1.0, "y": 0.0, "z": 0.0, "yaw_deg": 90.0, "stop_time": 2.0}
  ],
  "speed_params": {
    "max_linear_x": 0.5,
    "max_angular_z": 0.4
  },
  "auto_start": true
}
```

也支持 YAML 文件中使用：

```yaml
route:
  polyline:
    - {x: 0.0, y: 0.0, z: 0.0}
    - {x: 1.0, y: 0.0, z: 0.0}
stop_points:
  - {name: P0, x: 1.0, y: 0.0, z: 0.0, yaw_deg: 90.0, stop_time: 2.0}
```

发布示例：

```bash
python3 send.py route_bundle.yaml
python3 send.py route_bundle.yaml --no-auto-start
python3 send.py route_bundle.yaml --max-vx 0.4 --lookahead-distance 0.8
```

## 多楼层点云

页面左侧提供“楼层 / 高度过滤”：

- 勾选“只显示并选取当前高度层”后，点云显示、点云拾取、路线点击、路点点击和视图适配都会限制在当前 `Z 最小值 ~ Z 最大值` 范围内。
- “全部高度”会恢复完整点云显示。
- “下一低层 / 下一高层”会按当前层厚度平移高度窗口，适合快速切换楼层。

这个功能解决的是多楼层点云在 3D 投影中互相遮挡，导致低楼层点位难以选取的问题。跨楼层导航仍建议在路线中显式连接电梯、坡道或楼梯等层间通道。

## 地图工程文件

页面“地图”页提供：

- “打开3D点云地图”：读取原始点云。
- “保存地图工程”：导出一个 JSON 文件，包含当前采样点云和可行驶区域。
- “读取地图工程”：直接恢复点云显示和可行驶区域，不需要重新划分区域。

地图工程文件用于现场复用已经划好的区域。它保存的是 Web 端使用的采样点云，不替代原始高精度点云文件。

## 可行驶区域和自动规划

页面“规划”页提供“可行驶区域 / 自动规划”：

- “绘制区域”：在当前高度层里点击地图，绘制可行驶区域边界。
- “完成区域”：至少 3 个点后保存为一个可行驶多边形。
- “删除选中区域”：从区域列表里删除某一个可行驶区域。
- “标定中心线”：沿楼梯、坡道或走廊中心依次点击点云点。
- “生成区域”：按通道宽度把中心线扩张成可行驶区域。生成的区域会进入区域列表，可直接删除。
- “点击目标规划”：第一次点击终点位置，第二次点击终点朝向，系统用当前 `/pcl_pose` 作为起点自动规划路线。
- “规划分辨率”：A* 网格分辨率，越小越细但计算越慢。
- “机器人半径”：规划时会让路径离区域边界至少保持这个距离。

自动规划优先由 C++ `indoor_route_planner_node` 完成：把可行驶区域栅格化后做 8 邻域 A*，再做路径拉直和平滑，最后由 Python 后端写入现有 `route_polyline`。

规划不会把区域外的起点或终点吸附到区域内；如果当前位置或目标点不在可行驶区域内，会直接报错。

## 注意事项

- 修改系统参数里的话题名后，需要重启 `web_app.py` 才会重新创建 ROS2 订阅。
- 前端上传点云时默认 `sample_step=1`，不再主动抽稀；如果后续地图太大导致浏览器卡顿，再把该参数调大。
- 导航启动前必须先收到 `/pcl_pose` 定位，否则会提示“尚未定位”。
- 如果网页位置不更新，先检查 `/pcl_pose` 的实际类型和 QoS：`ros2 topic info -v /pcl_pose`。
