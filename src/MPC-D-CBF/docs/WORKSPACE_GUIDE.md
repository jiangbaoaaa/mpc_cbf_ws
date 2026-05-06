# 工作区与运行栈指南

## 1. 先把这个仓库看成什么

这个仓库不是一个单独的“算法脚本”，而是一整套 ROS Noetic 仿真与控制栈。  
它把一个动态避障实验拆成了 7 个主要包：

- `scene`：启动 Gazebo 世界、生成动态障碍物、把 Gazebo 真值转成 TF。
- `jackal/jackal_description`：定义 Jackal 机器人模型、传感器和 Gazebo 插件。
- `jackal/jackal_control`：定义底盘控制器、`/cmd_vel` 多路复用、可选 EKF。
- `local_map`：把点云转成局部地图，再从局部地图提取障碍物椭圆。
- `obs_param`：对障碍物做时序跟踪和未来预测。
- `global_path_publisher`：发布全局参考路径。
- `local_planner`：根据机器人状态、全局路径、障碍物预测结果做局部规划。

如果你想“优化算法”，本质上是在这几个包里分别动不同层：

- 仿真环境层：`scene` + `jackal/*`
- 感知层：`local_map`
- 运动估计 / 预测层：`obs_param`，以及部分 `jackal_control` / `scene`
- 避障与规划层：`local_planner`
- 参考路径层：`global_path_publisher`

## 2. 目录层级怎么理解

推荐把它放在 catkin 工作区中：

```text
~/catkin_ws/
  src/
    MPC-D-CBF/
      docs/
      global_path_publisher/
      jackal/
      local_map/
      local_planner/
      obs_param/
      scene/
      tools/
```

其中你真正需要重点理解的不是 `build/`、`devel/`，而是 `src/MPC-D-CBF/` 下面这些包和 `docs/`。

## 3. 运行链路总图

当前代码里的真实数据流可以概括成：

```text
scene/worlds/world.world
  -> scene/launch/start.launch
  -> Gazebo + Jackal + 动态障碍物

jackal_description/urdf/jackal.urdf.xacro
  -> 生成机器人模型和 Velodyne 传感器
  -> 发布 /velodyne_points

scene/src/getrobot_pose.cpp
  <- /base_pose_ground_truth
  -> TF: world -> base_link

local_map/src/local_map_pub.cpp
  <- /velodyne_points
  <- TF(world, base_link, velodyne)
  -> /local_map_pub/gridmap
  -> /local_map_pub/local_pcd
  -> /local_map_pub/ellipse_vis
  -> /local_map_pub/for_obs_track

obs_param/src/obs_kf.cpp
  <- /local_map_pub/for_obs_track
  -> /obs_predict_pub
  -> /obs_predict_vis_pub

global_path_publisher/src/global_path_publisher.cpp
  -> /global_path

local_planner/scripts/my_local_planner.py
  <- /curr_state
  <- /obs_predict_pub
  <- /global_path
  -> /local_plan
  -> /local_path
  -> /pub_path_vis
  -> /cmd_move

local_planner/scripts/controller.py
  <- TF(world, base_link)
  <- /local_plan
  -> /curr_state
  -> /cmd_vel

jackal_control
  <- /cmd_vel
  -> Jackal 差速底盘
```

这张图非常重要，因为你以后改某个算法时，最好先确认它属于哪一层，而不是上来直接改规划器。

## 4. 为什么原来没有在环境启动时直接打开 RViz 和 Gazebo GUI

原来的设计思路是把“环境侧”和“规划侧”拆开：

- `sim_environment.launch` 只负责环境、障碍物、感知、预测和全局路径，默认不开 GUI。
- `visualization.launch` 负责 Gazebo GUI 和 RViz。
- `planner_only.launch` 只负责规划器和控制器。

这样设计的好处是：

- 环境侧可以无头运行，更省资源。
- 如果规划器崩了，只需要重启规划侧，不需要重启 Gazebo 世界。
- Gazebo GUI 和 RViz 默认被视为“规划调试工具”，而不是环境本体的一部分。

但你说的问题也是成立的：  
当你主要在改 `local_map`、动态障碍物或者场景本身时，先开环境却看不到 GUI，确实不方便。

所以我已经把启动流程改成既保留旧方式，也支持你要的“两步式”方式。

## 5. 现在支持的启动方式

### 方式 A：两步式调试工作流

这个方式最适合你现在的需求：  
先启动“障碍物环境并可视化”，确认局部地图、动态障碍物和预测是否正常；然后再单独启动规划器。

### 第一步：启动环境 + 可视化，但不启动规划器

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch scene env_visualization.launch
```

如果你要直接使用“最后一个动态障碍物横穿”的专用实验场景，可以改为：

```bash
roslaunch scene env_visualization_side_static.launch
```

这个 launch 会启动：

- Gazebo 服务器 `gzserver`
- Gazebo GUI `gzclient`
- RViz
- Jackal 模型描述和底盘控制
- 机器人 spawn
- `local_map`
- `movetest_node`
- `obs_kf`
- `global_path_pub`

这个阶段你可以先专注检查：

- 点云是否正常
- 局部地图是否稳定
- 椭圆拟合是否合理
- 动态障碍物运动是否符合预期
- 预测椭圆是否正常发布

### 第二步：开始规划

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch scene planner_only.launch
```

这个 launch 只启动：

- `local_planner`
- `controller`

这样你可以在不重启环境和可视化的前提下，反复重启规划器。

### 方式 B：基础环境启动

### 终端 1：环境侧，无可视化

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch scene sim_environment.launch
```

这个 launch 会启动：

- Gazebo 服务器 `gzserver`
- Jackal 模型描述和底盘控制
- 机器人 spawn
- `local_map`
- `movetest_node` 动态障碍物
- `obs_kf` 障碍物预测
- `global_path_pub` 全局路径

### 方式 C：单命令全启动

```bash
roslaunch scene full_stack.launch
```

这个 launch 现在等价于：

- 先启动 `env_visualization.launch`
- 再启动 `planner_only.launch`

## 6. 按优化目标快速定位文件

| 优化目标 | 第一优先修改文件 | 第二优先修改文件 | 说明 |
| --- | --- | --- | --- |
| 仿真场景、障碍物布局、物理参数 | `scene/worlds/world.world` | `scene/launch/start.launch`、`scene/launch/movetest_node.launch`、`scene/src/movetest.cpp` | 决定世界、障碍物初始状态、速度与运动方式 |
| 传感器模型、激光雷达位置、仿真真值接口 | `jackal/jackal_description/urdf/jackal.urdf.xacro` | `jackal/jackal_description/urdf/jackal.gazebo`、`jackal/jackal_description/launch/description.launch` | 决定点云来源、IMU/GPS/真值插件 |
| 局部感知、障碍物提取 | `local_map/src/local_map_pub.cpp` | `local_map/include/DBSCAN.hpp`、`local_map/include/ellipse.hpp`、`local_map/include/KM.hpp`、`local_map/launch/for_simulation.launch` | 点云裁剪、栅格化、障碍物检测、聚类、椭圆拟合、ID 关联都在这里 |
| 障碍物运动估计 / 预测 | `obs_param/src/obs_kf.cpp` | `obs_param/include/kalman.h`、`obs_param/src/kalman.cpp` | 这里是障碍物的滤波状态、协方差和多步预测 |
| 机器人状态估计 | `local_planner/scripts/controller.py` | `scene/src/getrobot_pose.cpp`、`jackal/jackal_control/config/robot_localization.yaml`、`jackal/jackal_control/launch/control.launch` | 当前默认更接近“仿真真值 + TF 读取”，如果要做更真实定位需要改这些文件 |
| 避障约束、MPC 目标函数、轨迹生成 | `local_planner/scripts/my_local_planner.py` | `local_planner/config/my_planner_params.yaml`、`local_planner/launch/local_planner.launch`、`local_planner/scripts/controller.py` | 当前默认运行的是你维护的 `my_local_planner` |
| 全局路径 / 参考线 | `global_path_publisher/src/global_path_publisher.cpp` | `global_path_publisher/launch/pub_global_path.launch` | 当前全局路径只是起点到终点的直线 |
| 底盘动力学限制、速度边界 | `jackal/jackal_control/config/control.yaml` | `local_planner/scripts/controller.py` | 影响规划器给出的速度在仿真里能否真实执行 |

## 7. 当前仓库里最容易混淆的几个点

### 6.1 当前 launch 启动的是哪个规划器

`local_planner/launch/local_planner.launch` 现在启动的是：

- `my_local_planner.py`
- `controller.py`

这意味着：

- 如果你要改当前默认运行的规划器，直接看 `my_local_planner.py`。
- `planner_only.launch` 会先加载 `local_planner/config/my_planner_params.yaml`，再启动规划器。
- `adaptive_local_planner.py` 目前不在默认启动链里。

### 6.2 感知和预测是两层，不要混在一起

- `local_map` 做的是“当前帧障碍物检测与几何建模”。
- `obs_param` 做的是“障碍物随时间的滤波与未来预测”。

所以如果你感觉障碍物位置“当前就不准”，优先查 `local_map`。  
如果当前帧位置准，但未来轨迹不准，优先查 `obs_param`。

### 6.3 机器人状态来源有两条线

当前默认是：

- Gazebo 通过 `jackal.gazebo` 发布 `/base_pose_ground_truth`
- `scene/src/getrobot_pose.cpp` 把它广播成 TF
- `controller.py` 通过 TF 读 `world -> base_link`
- `controller.py` 再发布 `/curr_state`

如果你想从“理想真值”切到“更真实的定位估计”，就要把关注点转到：

- `jackal_control/config/robot_localization.yaml`
- `jackal_control/launch/control.launch`

### 7.4 仿真环境和可视化为什么被拆成多个 launch

这不是冗余，而是为了把：

- 世界、点云、障碍物、预测
- 规划器、RViz、Gazebo GUI

拆开，便于调试与重启。

现在保留的主要入口是：

- `env_visualization.launch`：环境 + 可视化，不启动规划器
- `env_visualization_side_static.launch`：新场景，环境 + 可视化，不启动规划器
- `planner_only.launch`：只启动规划器
- `visualization.launch`：只负责 Gazebo GUI 和 RViz，供其他 launch 复用

### 7.5 `my_local_planner` 的参数现在在哪里改

现在默认规划器是 `my_local_planner.py`，参数入口优先看：

- `local_planner/config/my_planner_params.yaml`

这里已经暴露出大部分常用 ROS 参数，例如：

- `my_local_planner/horizon`
- `my_local_planner/dt`
- `my_local_planner/gamma_k`
- `my_local_planner/v_max`
- `my_local_planner/v_min`
- `my_local_planner/omega_max`

如果你要改 MPC 目标函数、障碍物约束构造、求解失败回退等逻辑，再去看：

- `local_planner/scripts/my_local_planner.py`

## 8. 如果你想开始优化，建议的阅读顺序

建议按下面顺序读代码，而不是按目录顺序平铺看：

1. 先读 [ALGORITHM_NOTES.md](./ALGORITHM_NOTES.md)，理解算法链路和每个优化方向对应的文件。
2. 再读 [FILE_GUIDE.md](./FILE_GUIDE.md)，逐个理解关键文件职责。
3. 真正开始改代码时，优先看：
   - 仿真：`scene/*` 和 `jackal/*`
   - 感知：`local_map/src/local_map_pub.cpp`
   - 预测：`obs_param/src/obs_kf.cpp`
   - 规划：`local_planner/scripts/my_local_planner.py`

## 9. 一个最实用的修改原则

以后每次想“优化算法”，都先问自己这 3 个问题：

1. 问题出在“输入不对”还是“决策不对”？
2. 如果输入不对，是传感器模型、局部感知，还是运动预测不对？
3. 如果决策不对，是参考路径、避障约束、目标函数，还是底盘执行不匹配？

把这 3 个问题答清楚，再去改对应文件，效率会高很多。
