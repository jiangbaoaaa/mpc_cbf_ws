# 逐文件说明

## 1. 使用说明

这份文档的目标不是复述目录名，而是帮你快速判断：

- 这个文件到底负责什么
- 想优化哪一类功能时为什么要改它
- 它在整条链路中处于什么位置

说明范围以“会影响运行逻辑或算法理解的文本文件”为主。  
像 `meshes/*.stl` 这样的几何资源文件不会逐个展开算法含义，但会在最后说明其类别作用。

## 2. 根目录文件

### `README.md`

仓库总说明。  
适合第一次进入项目时看，用来理解：

- 这是一个 catkin 工作区下的 ROS 包集合
- 依赖怎么装
- 仿真如何启动
- `docs` 里有哪些文档

它不是算法实现文件，但它是整个仓库的导航页。

### `tools/bootstrap_catkin_ws.sh`

一个辅助脚本，用于把当前仓库软链接进一个新的 catkin 工作区。  
它不影响算法本身，但能快速准备开发环境。

适合修改的情况：

- 你想支持不同工作区路径
- 你想自动化初始化更多依赖

## 3. `scene` 包

### `scene/CMakeLists.txt`

定义 `scene` 包里要编译的可执行节点：

- `movetest_node`
- `getrobot_pose`

如果你新增环境节点或改了源码文件名，需要同步改这里。

### `scene/package.xml`

声明 `scene` 包的 ROS 依赖。  
如果你在 `scene` 中新增了新的 ROS 消息类型、Gazebo 接口或库依赖，需要同步更新它。

### `scene/launch/start.launch`

环境侧底层启动入口。

它负责：

- 启动 `gzserver`
- 打开 `/use_sim_time`
- 视情况启动 `gzclient`
- 启动 `getrobot_pose`

如果你要切换世界文件、给 Gazebo 增加参数、加入新的基础环境节点，改这里。

### `scene/launch/sim_environment.launch`

当前最重要的环境总入口。

它负责把下面这些模块串起来：

- `start.launch`
- `jackal_description`
- `jackal_control`
- 机器人 spawn
- `local_map`
- 可选障碍物节点
- `obs_kf`
- `global_path_pub`

如果你要换环境侧模块组合，基本会改这个文件。

### `scene/launch/visualization.launch`

纯可视化入口。

它负责：

- 启动 Gazebo GUI
- 启动 RViz

这个文件的意义是把可视化从规划逻辑里拆出来，便于复用。

### `scene/launch/env_visualization.launch`

新的“两步式调试”第一步入口。

它负责：

- 启动完整环境侧
- 启动 Gazebo GUI
- 启动 RViz

但不启动局部规划器。

如果你在调：

- 动态障碍物
- `local_map`
- 感知与预测链
- 场景和传感器模型

这个 launch 比原来的 `sim_environment.launch` 更方便。

### `scene/launch/env_visualization_side_static.launch`

这是目前用于“末端横穿动态障碍物”实验的专用入口。

它负责：

- 启动 `side_static_dynamic.world`
- 启动 Gazebo GUI
- 启动 RViz
- 不启动规划器

如果你要复现实验里“最后一个动态障碍物”的效果，优先用它。

### `scene/launch/env_visualization_blind_wall_static.launch`

这是“墙体遮挡 + 墙后静态障碍”的专用入口。

它负责：

- 启动 `blind_wall_static.world`
- 启动 Gazebo GUI
- 启动 RViz
- 不启动规划器

适合先观察有限视野下：

- 点云被墙体遮挡后的缺失情况
- 隐藏静态障碍何时进入视野
- 局部地图和椭圆拟合何时开始反映遮挡后的障碍

### `scene/launch/planner_only.launch`

新的“两步式调试”第二步入口。

它只负责启动：

- `local_planner`
- `controller`

适合在环境已经跑起来后，单独开始或重启规划器。

### `scene/launch/movetest_node.launch`

动态障碍物参数文件。

它定义了 8 个障碍物的：

- 初始位置
- 高度
- 平移速度

如果你想快速改障碍物布局，而不改 C++ 逻辑，直接改这里。

### `scene/src/movetest.cpp`

动态障碍物节点主程序。

作用：

- 等待 `/cmd_move`
- 从参数读障碍物初始状态
- 周期性更新障碍物位置
- 调用 Gazebo 服务写回模型状态

如果你想改障碍物运动模型，这是首要文件。

### `scene/include/moving_cylinder.hpp`

单个障碍物对象的简单封装。

它负责：

- 自动生成 Gazebo 模型名
- 保存位姿与速度
- 执行一步位置更新

这是 `movetest.cpp` 的辅助文件，不是独立节点。

### `scene/src/getrobot_pose.cpp`

把 Gazebo 的 `/base_pose_ground_truth` 转成 TF。

作用：

- 订阅真值里程计
- 提取 pose
- 广播 `world -> base_link` 变换

当前很多状态链都依赖这个 TF，所以它对“机器人状态来源”非常关键。

### `scene/worlds/world.world`

Gazebo 世界文件。

决定：

- 世界物理参数
- 场景中有哪些模型
- 动态障碍物默认模型和初始摆放
- 摄像机视角

做仿真环境优化时，它几乎总是必改文件。

### `scene/rviz_config/simulation.rviz`

RViz 配置文件。  
只影响可视化显示，不直接改变算法。

适合修改的情况：

- 想显示更多 topic
- 想调整颜色、图层、相机视角
- 想更方便调试局部地图、预测椭圆、局部轨迹

## 4. `local_map` 包

### `local_map/CMakeLists.txt`

定义 `local_map_pub` 的编译规则和依赖库。

如果你：

- 新增源文件
- 新增 PCL、CGAL、ROS 依赖
- 改可执行目标名

就要改它。

### `local_map/package.xml`

声明 `local_map` 依赖。  
和 `CMakeLists.txt` 一样，属于包级配置文件。

### `local_map/launch/for_simulation.launch`

局部地图节点启动与参数入口。

最常用的实验参数在这里：

- 地图尺寸
- 分辨率
- 障碍物高度阈值
- 台阶高度阈值
- DBSCAN 参数

做感知调参时优先改它。

### `local_map/src/local_map_pub.cpp`

感知层主逻辑文件，是整个仓库里最值得精读的文件之一。

它负责：

- 从 `/velodyne_points` 取点云
- 利用 TF 转到 `world`
- 对点云裁剪与降采样
- 投影到局部网格地图
- 对高度图插值、膨胀
- 提取障碍物候选点
- 调用 DBSCAN 聚类
- 调用椭圆拟合
- 调用 ID 关联
- 发布当前障碍物椭圆给预测器

如果你在优化感知算法，通常 70% 的时间都会花在这个文件上。

### `local_map/include/DBSCAN.hpp`

一个简化版 DBSCAN 实现。

作用：

- 接收障碍物候选点
- 按欧式距离聚类
- 标记噪声点与簇编号

如果你要换聚类方法或调聚类逻辑，改它。

### `local_map/include/ellipse.hpp`

障碍物几何建模文件。

作用：

- 定义 `Ellipse`
- 调用 CGAL 最小外接椭圆
- 为退化情况构造备用椭圆
- 计算椭圆长短轴变化方差
- 发布椭圆可视化

如果你要把椭圆改成更复杂的障碍物表达，改它。

### `local_map/include/KM.hpp`

障碍物 ID 跟踪文件。

作用：

- 把当前帧椭圆和上一帧椭圆做匹配
- 复用旧标签
- 尝试从历史列表中恢复曾经出现过的障碍物

这个文件解决的是“谁是谁”的问题，不是“未来怎么走”的问题。

### `local_map/include/third_party/Hungarian.hpp`

第三方 Hungarian 匹配算法实现。  
通常不需要改。

只有在以下情况下才考虑动它：

- 想优化分配算法性能
- 想检查底层最优分配行为
- 想替换成本矩阵求解器

## 5. `obs_param` 包

### `obs_param/CMakeLists.txt`

定义 `obs_kf` 节点编译规则。

如果你新增预测器文件、重命名目标、引入新库，改这里。

### `obs_param/package.xml`

`obs_param` 的依赖声明文件。  
属于包级配置，不是算法核心。

### `obs_param/include/kalman.h`

卡尔曼滤波类的声明文件。

定义了：

- 状态向量
- 控制向量
- 观测向量
- 系统矩阵与协方差矩阵
- 标准预测 / 更新接口

如果你想扩展滤波器框架，从这里入手最清楚。

### `obs_param/src/kalman.cpp`

卡尔曼滤波类的标准实现。

包括：

- 状态预测
- 协方差预测
- 观测残差
- 卡尔曼增益
- 状态更新
- 协方差更新

这是通用数学层，通常不会频繁改业务逻辑。

### `obs_param/src/obs_kf.cpp`

障碍物预测主逻辑文件，是预测层的核心。

它负责：

- 接收当前帧障碍物椭圆观测
- 为每个 label 维护一套滤波状态
- 根据观测方差调整测量协方差
- 向未来滚动预测 `N=25` 步
- 对未来椭圆做协方差膨胀
- 发布给局部规划器

如果你要改障碍物运动估计，这个文件是第一入口。

## 6. `local_planner` 包

### `local_planner/CMakeLists.txt`

定义 Python 脚本的安装规则。

现在会安装：

- `my_local_planner.py`
- `local_planner.py`
- `controller.py`

同时会安装 `launch/` 和 `config/`。

### `local_planner/package.xml`

`local_planner` 的包依赖声明文件。  
如果你给规划器加了新的 ROS 依赖或 Python 侧接口，需要同步更新。

### `local_planner/launch/local_planner.launch`

局部规划侧 launch 入口。

当前它默认启动：

- `my_local_planner.py`
- `controller.py`

它会先加载 `config/my_planner_params.yaml`。  
`adaptive_local_planner.py` 目前不在默认启动链里。  
如果你要改局部规划器，就直接改 `my_local_planner.py`。

### `local_planner/scripts/my_local_planner.py`

当前默认运行的局部规划器。

作用：

- 从全局路径中截取局部目标
- 读取未来障碍物椭圆
- 建立非线性 MPC 问题
- 用椭圆安全函数构建 DCBF 约束
- 用 IPOPT 求解
- 发布局部轨迹和控制序列

如果你想研究当前实际在跑的 MPC-DCBF 逻辑，这个文件最重要。

### `local_planner/config/my_planner_params.yaml`

默认规划器的大部分调参项都已经抽到了这个 YAML。

最常改的是：

- `my_local_planner/horizon`
- `my_local_planner/dt`
- `my_local_planner/gamma_k`
- `my_local_planner/v_max`
- `my_local_planner/v_min`
- `my_local_planner/omega_max`

如果你要改代价函数、约束形式或回退逻辑，再去改 `my_local_planner.py`。

### `local_planner/scripts/controller.py`

规划结果执行桥。

作用：

- 订阅 `/local_plan`
- 取第一步控制量发给 `/cmd_vel`
- 从 TF 计算机器人当前状态
- 发布 `/curr_state`

它把“规划器输出”和“机器人执行”接起来，所以虽然代码短，但影响很大。

### `local_planner/scripts/controller.py` 里值得特别记住的点

- 它不是纯控制器，也负责状态发布。
- 它本地缓存 `N = 10`，和规划器的 `N = 25` 不一致。
- 它最终只执行第一步控制，这是 MPC 的 receding horizon 方式。

## 7. `global_path_publisher` 包

### `global_path_publisher/CMakeLists.txt`

定义 `global_path_pub` 的编译方式。  
如果你新增源文件或换成别的路径生成器，需要改它。

### `global_path_publisher/package.xml`

依赖声明文件。  
属于包配置，不是路径算法本体。

### `global_path_publisher/launch/pub_global_path.launch`

全局路径发布节点的参数入口。

当前只配置了：

- 起点
- 终点

如果你只想快速换实验目标点，直接改它即可。

### `global_path_publisher/src/global_path_publisher.cpp`

全局路径主逻辑文件。

当前逻辑非常简单：

- 读取起点和终点
- 以固定步长插值成直线路径
- 周期性发布 `/global_path`

如果你想引入真正的全局规划器，它是首要替换文件。

## 8. `jackal/jackal_control` 包

### `jackal/jackal_control/CMakeLists.txt`

上游 Jackal 控制包的构建文件。  
这个包本身几乎都是 launch 和 yaml，不包含你自己的算法节点。

### `jackal/jackal_control/package.xml`

上游 Jackal 控制包依赖声明。

### `jackal/jackal_control/launch/control.launch`

底盘控制入口。

作用：

- 加载 `control.yaml`
- 启动差速控制器
- 可选启动 `robot_localization`
- 启动 `twist_mux`

如果你要控制真实程度更高的底盘闭环，这个文件非常重要。

### `jackal/jackal_control/launch/teleop.launch`

遥控与交互式控制入口。

作用：

- 启动 PS3/PS4 手柄
- 启动交互式 marker 控制

对算法本身影响较小，但对调试很有用。

### `jackal/jackal_control/config/control.yaml`

底盘控制器参数文件。

影响：

- 速度上限
- 加速度上限
- 轮距 / 轮半径缩放
- 差速驱动执行特性

如果规划器速度边界和这里不一致，就可能出现“规划可行、执行失真”的现象。

### `jackal/jackal_control/config/robot_localization.yaml`

EKF 融合参数文件。

如果你想摆脱“直接用真值”的状态链，想做更真实定位，这个文件必看。

### `jackal/jackal_control/config/twist_mux.yaml`

速度指令多路复用规则。

它定义了不同控制源的优先级：

- joystick
- 蓝牙手柄
- interactive marker
- external `/cmd_vel`

当前算法输出走的是 `external` 这一条。

### `jackal/jackal_control/config/teleop_ps3.yaml`

PS3 手柄按键映射。  
只影响人工遥控，不影响避障算法逻辑。

### `jackal/jackal_control/config/teleop_ps4.yaml`

PS4 手柄按键映射。  
和 `teleop_ps3.yaml` 类似，属于调试辅助文件。

### `jackal/jackal_control/CHANGELOG.rst`

上游包版本日志。  
不参与算法运行。

## 9. `jackal/jackal_description` 包

### `jackal/jackal_description/CMakeLists.txt`

上游机器人描述包的构建文件。  
主要负责安装 `meshes`、`launch`、`urdf` 和脚本。

### `jackal/jackal_description/package.xml`

机器人描述包依赖声明。  
当你引入新的传感器 xacro 或上游描述包时，需要同步看它。

### `jackal/jackal_description/launch/description.launch`

机器人描述入口。

它做的事情是：

- 读取配置脚本
- 运行 xacro
- 把生成的 URDF 放到 `robot_description`
- 启动 `robot_state_publisher`

如果你改了机器人模型但 launch 没有引用到正确配置，这里就会出问题。

### `jackal/jackal_description/urdf/jackal.urdf.xacro`

机器人本体与传感器模型主文件。

它决定：

- 车体尺寸和轮子
- 激光雷达挂载
- 基本 link / joint
- Gazebo 插件文件引用
- 附件 xacro 引用

它是仿真环境与传感器模型最关键的文件之一。

### `jackal/jackal_description/urdf/jackal.gazebo`

Gazebo 插件配置文件。

它定义：

- Gazebo ros_control 插件
- IMU 插件
- GPS 插件
- `/base_pose_ground_truth` 插件

所以它直接决定了仿真中哪些 topic 会被发布出来。

### `jackal/jackal_description/urdf/accessories.urdf.xacro`

附件总入口 xacro。

作用：

- 汇总各种传感器和支架宏
- 通过环境变量决定是否启用某类附件

如果你以后想把仿真车改成更复杂的多传感器平台，这个文件值得看。

### `jackal/jackal_description/urdf/accessories/*.urdf.xacro`

这批文件都是“某种附件或支架的挂载宏”。

- `bridge_plate.urdf.xacro`：桥式安装板
- `camera_mount.urdf.xacro`：相机支架
- `hdl32_mount.urdf.xacro`：HDL-32E 激光雷达支架
- `hokuyo_ust10.urdf.xacro`：Hokuyo UST10 支架
- `hokuyo_utm30.urdf.xacro`：Hokuyo UTM30 支架
- `novatel_smart6.urdf.xacro`：Smart6 GNSS 描述
- `novatel_smart7.urdf.xacro`：Smart7 GNSS 描述
- `sick_lms1xx_inverted_mount.urdf.xacro`：倒装 LMS1xx 支架
- `sick_lms1xx_upright_mount.urdf.xacro`：正装 LMS1xx 支架
- `standoffs.urdf.xacro`：立柱 / 垫高件
- `stereo_camera_mount.urdf.xacro`：双目相机支架
- `vlp16_mount.urdf.xacro`：VLP16 支架

这些文件大多不直接影响你当前算法，但会影响传感器安装位姿和仿真模型结构。

### `jackal/jackal_description/urdf/configs/base`

最基础的 Jackal 配置，不启用额外附件。  
`description.launch` 默认会读取它。

### `jackal/jackal_description/urdf/configs/front_laser`

一个示例配置，启用前向 2D 激光。  
适合理解“配置文件如何通过环境变量打开某个传感器”。

### `jackal/jackal_description/urdf/configs/front_flea3`

启用前置 Flea3 相机的配置模板。

### `jackal/jackal_description/urdf/configs/front_bumblebee2`

启用 Bumblebee2 相机的配置模板。

### `jackal/jackal_description/urdf/configs/*.bat`

对应 Windows 环境下的配置文件。  
在 Linux ROS Noetic 仿真里通常不会用到。

### `jackal/jackal_description/scripts/env_run`

一个小脚本，用于先 `source` 配置文件，再执行 xacro 命令。  
`description.launch` 会用它把 `urdf/configs/*` 里的环境变量带进去。

### `jackal/jackal_description/scripts/env_run.bat`

`env_run` 的 Windows 版本。

### `jackal/jackal_description/README.md`

上游 Jackal 机器人描述包说明。  
适合了解有哪些官方支持的传感器和环境变量。

### `jackal/jackal_description/CHANGELOG.rst`

上游版本日志，不参与算法运行。

## 10. `docs` 目录

### `docs/WORKSPACE_GUIDE.md`

说明整套工作区怎么启动、怎么理解运行链路、如何按优化目标找入口文件。

### `docs/ALGORITHM_NOTES.md`

从“仿真环境 / 感知 / 运动估计 / 避障 / 全局路径”几层来解释代码。

### `docs/FILE_GUIDE.md`

也就是当前这份文件，给你逐文件解释项目结构。

## 11. 其他文件类别

### `jackal/LICENSE`

上游 Jackal 代码许可证文件。  
和算法实现无关，但和开源使用方式有关。

### `jackal/jackal_description/meshes/*.stl`

机器人与附件的三维模型资源文件。

它们的作用是：

- 提供外观显示
- 提供部分碰撞几何参考

通常不需要改，除非你要换机器人外形或附件。

## 12. 如果你现在就要开始改代码，最建议先看的 10 个文件

如果只选 10 个最关键文件，我建议按下面顺序读：

1. `scene/launch/sim_environment.launch`
2. `scene/worlds/world.world`
3. `jackal/jackal_description/urdf/jackal.urdf.xacro`
4. `jackal/jackal_description/urdf/jackal.gazebo`
5. `local_map/src/local_map_pub.cpp`
6. `local_map/include/ellipse.hpp`
7. `obs_param/src/obs_kf.cpp`
8. `local_planner/launch/local_planner.launch`
9. `local_planner/scripts/my_local_planner.py`
10. `local_planner/scripts/controller.py`

看完这 10 个文件，你基本就能把整个系统串起来。
