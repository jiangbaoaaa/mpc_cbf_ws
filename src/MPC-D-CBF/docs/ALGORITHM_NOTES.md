# 算法说明与优化入口

## 1. 这套代码里的算法链路到底是什么

从运行逻辑上看，这套系统不是“一个避障算法”，而是 5 层串联：

1. 仿真环境与传感器层  
   Gazebo 世界、Jackal 机器人、Velodyne 点云、真值位姿、动态障碍物。
2. 感知建图层  
   点云裁剪、栅格化、高度图、梯度图、障碍物候选点、DBSCAN 聚类、椭圆拟合、障碍物 ID 关联。
3. 运动估计与预测层  
   对每个障碍物做卡尔曼滤波，并沿规划时域预测未来椭圆。
4. 参考路径层  
   提供全局路径，再截取成局部参考轨迹。
5. 规划与控制层  
   根据机器人状态、局部参考轨迹和障碍物预测结果，求解局部控制，再通过 `/cmd_vel` 执行。

如果你要优化算法，最关键的不是“多写一个新方法”，而是先定位你到底要换哪一层。

## 2. 仿真环境优化时，应该改哪些文件

仿真环境优化通常包括：

- 障碍物形状、数量、位置、速度
- Gazebo 物理参数
- 机器人模型
- 传感器安装位置与传感器模型
- 仿真真值与噪声模型

### 2.1 场景本体

#### `scene/worlds/world.world`

这是 Gazebo 世界文件，是仿真环境优化的第一入口。  
你如果要改下面这些内容，优先改它：

- 地面与静态模型
- 障碍物初始摆放
- Gazebo 物理参数，例如步长、实时因子
- 场景视觉效果

如果你想把当前的简单圆柱场景换成更复杂的场景，这是最先动的文件。

#### `scene/launch/start.launch`

这个文件负责启动 `gzserver`，打开仿真时间，并启动 `getrobot_pose`。  
它更像环境侧总入口。

适合修改的内容：

- 启动哪个 `.world`
- 是否打开 Gazebo GUI
- 是否额外挂更多环境节点

需要注意：它的默认 `world` 参数写的是 `map.world`，但 `sim_environment.launch` 实际上传入的是 `world.world`。

#### `scene/launch/sim_environment.launch`

这是当前最重要的环境侧 launch。  
它把下面这些部件串起来：

- `start.launch`
- `jackal_description`
- `jackal_control`
- 机器人 spawn
- `local_map`
- `movetest_node`
- `obs_kf`
- `global_path_publisher`

如果你要新增环境节点、替换障碍物运动节点、换机器人描述或换路径源，这里通常也要一起改。

### 2.2 动态障碍物

#### `scene/launch/movetest_node.launch`

这个文件定义了 8 个圆柱障碍物的初始位置和速度：

- `x0/y0/z0/vx0/vy0`
- `x1/y1/z1/vx1/vy1`
- ...

如果你只是想快速改实验布局，而不改 C++ 逻辑，直接改这里最快。

#### `scene/src/movetest.cpp`

这个文件是“障碍物怎么动”的真实逻辑入口。  
它做的事情很简单：

- 订阅 `/cmd_move`
- 从参数服务器读每个圆柱的初始位置与速度
- 每 0.01 s 更新一次位置
- 通过 `/gazebo/set_model_state` 把障碍物写回 Gazebo

如果你想优化动态障碍物模型，例如：

- 让障碍物按轨迹走而不是匀速
- 增加转向逻辑
- 加速度模型
- 随机扰动
- 与机器人交互

这个文件就是第一修改点。

#### `scene/include/moving_cylinder.hpp`

这是 `movetest.cpp` 的配套头文件。  
它不是算法核心，但封装了单个圆柱对象的状态：

- 模型名
- 位姿
- 速度
- 单步更新规则

如果你想把圆柱从“二维匀速平移”改成更复杂的运动体，通常要和 `movetest.cpp` 一起改。

### 2.3 机器人模型与传感器

#### `jackal/jackal_description/urdf/jackal.urdf.xacro`

这是仿真机器人模型最关键的文件之一。  
当前文件里直接启用了一个 3D 激光雷达：

- topic：`/velodyne_points`
- 安装在 `base_link`
- 高度约 `0.5 m`

如果你要优化感知输入质量，或者改变传感器配置，优先改这里：

- 激光雷达类型
- 激光雷达安装高度/姿态
- 机器人几何尺寸
- 轮子、底盘、碰撞体

特别注意：当前注释里写了 VLP-16 的备选，但实际启用的是 `HDL-32E` 的 xacro 宏。

#### `jackal/jackal_description/urdf/jackal.gazebo`

这个文件定义 Gazebo 插件，是“仿真数据从哪里来”的核心：

- `gazebo_ros_control`
- IMU 插件
- GPS 插件
- `p3d` 真值插件，发布 `/base_pose_ground_truth`

如果你要优化：

- IMU/GPS 噪声
- 仿真真值输出
- 机器人在 Gazebo 里的控制接口

就要改这里。

#### `jackal/jackal_control/config/control.yaml`

这个文件虽然不直接生成避障算法，但它决定了底盘执行边界：

- 最大线速度
- 最大角速度
- 最大加速度
- 轮距与轮半径缩放
- 差速控制器参数

如果规划器看起来“算得出来，但机器人执行起来不对”，就要检查它和规划器里的速度边界是否一致。

## 3. 感知算法优化时，应该改哪些文件

这里的“感知算法”在当前仓库里主要指：

- 点云预处理
- 局部地图构建
- 障碍物提取
- 聚类
- 椭圆建模
- 障碍物 ID 跟踪

### 3.1 感知主入口

#### `local_map/src/local_map_pub.cpp`

这是整个感知链的绝对主文件。  
绝大多数感知层优化，最后都会落到这里。

它内部完成了下面这些步骤：

1. 从 `/velodyne_points` 读取点云
2. 通过 TF 把点云从 `velodyne` 变换到 `world`
3. 对点云做 `PassThrough` 裁剪和体素降采样
4. 把点云投影成局部高度图
5. 对高度图做插值与膨胀
6. 通过梯度和台阶高度差提取障碍物候选点
7. 调用 DBSCAN 做聚类
8. 对每个聚类拟合椭圆
9. 调用 KM 做标签关联
10. 计算椭圆方差并发布给 `obs_param`

如果你要优化：

- 远处点云太稀疏
- 点云噪声太大
- 障碍物漏检或误检
- 聚类边界不稳定
- 椭圆拟合偏大或偏小
- 感知帧率过低

第一优先都是这个文件。

这个文件里最值得关注的参数和逻辑包括：

- `localmap_x_size` / `localmap_y_size`
- `resolution`
- `obs_height`
- `step_height`
- `DBSCAN_R`
- `DBSCAN_N`
- `block_size`
- `map_interpolation`
- `map_inflate`
- `gradient_map_processing`

### 3.2 聚类

#### `local_map/include/DBSCAN.hpp`

这里实现的是一个简单的 DBSCAN。  
如果你想优化：

- 聚类阈值策略
- 距离定义
- 噪声点处理
- 大小筛选
- 计算效率

就要改这里。

当前实现是平面欧式距离聚类，输入点是障碍物候选栅格索引，不是原始点云。

### 3.3 椭圆拟合与不确定性

#### `local_map/include/ellipse.hpp`

这个文件非常关键，因为它决定感知结果是怎样被压缩成规划器可用的障碍物模型。

它主要负责：

- 定义 `Ellipse` 数据结构
- 用 CGAL 最小外接椭圆拟合聚类点
- 对过多点做下采样
- 在退化情况下给一个备用椭圆
- 计算 `a/b` 变化方差
- 生成 RViz 可视化

如果你要优化：

- 从椭圆换成更复杂几何体
- 引入椭圆置信区间
- 用更鲁棒的拟合方式
- 调整视觉上看到的椭圆大小

这个文件就是第一修改点。

### 3.4 障碍物 ID 跟踪

#### `local_map/include/KM.hpp`

这个文件做的不是动力学预测，而是“当前帧障碍物和上一帧障碍物怎么对上号”。  
它用的是 Hungarian 匹配，并维护了一个历史列表。

如果你感觉：

- 同一个障碍物 label 经常跳变
- 障碍物短暂消失又出现后 ID 乱了
- 近距离交叉时匹配错误

应该优先改这里，而不是先改卡尔曼滤波。

#### `local_map/include/third_party/Hungarian.hpp`

这是第三方 Hungarian 算法实现。  
通常不需要改，除非你明确想替换底层分配算法或者研究性能瓶颈。

### 3.5 感知参数入口

#### `local_map/launch/for_simulation.launch`

这个文件是感知参数的快速调节点。  
如果你只是想试验不同阈值，先改它最快：

- `localmap_x_size`
- `localmap_y_size`
- `resolution`
- `obs_height`
- `step_height`
- `DBSCAN_R`
- `DBSCAN_N`

一般建议先在这里调参数，确定方向后再回到 `local_map_pub.cpp` 改逻辑。

## 4. 运动估计 / 预测优化时，应该改哪些文件

这里有两种“运动估计”很容易混淆：

1. 障碍物运动估计 / 预测
2. 机器人自身位姿估计

这两类文件不在同一个包里。

### 4.1 障碍物运动估计 / 预测

#### `obs_param/src/obs_kf.cpp`

这是障碍物预测的主文件。  
它读取 `local_map` 发布的当前椭圆观测，然后对每个 obstacle label 维护一个滤波器。

它负责：

- 定义障碍物观测格式
- 维护每个障碍物的 `param_list`
- 调用卡尔曼预测 / 更新
- 基于协方差膨胀未来椭圆
- 发布 `N=25` 步预测结果

如果你要优化：

- 状态维度
- 过程模型
- 观测模型
- 过程噪声 / 测量噪声
- 多步预测保守度
- 不确定性感知安全边界

都应该先改这个文件。

当前实现里最值得重点看的位置有：

- `N = 25`
- `T = 0.1`
- `A` 矩阵
- `R` 的构造
- 根据 `mea_cov` 动态调整测量协方差
- `pred.a` / `pred.b` 中加入 `pos_conf + ab_conf`

#### `obs_param/include/kalman.h`

这个文件定义了卡尔曼滤波类的数据结构。  
如果你要从线性 KF 改成：

- EKF
- UKF
- IMM
- 常加速度模型
- 学习式预测器的统一接口

通常先从这里改类接口。

#### `obs_param/src/kalman.cpp`

这里实现标准卡尔曼预测与更新。  
如果你只是换 `A/Q/R/H`，多数情况只改 `obs_kf.cpp` 即可。  
如果你要换滤波器数学形式，才需要直接改这里。

### 4.2 机器人自身位姿估计

#### `scene/src/getrobot_pose.cpp`

这个文件把 `/base_pose_ground_truth` 转成 TF 广播。  
它本质上使用的是 Gazebo 真值。

如果你做的是：

- 更真实的定位仿真
- 带噪位姿估计
- 从“真值驱动”切成“里程计 / IMU / EKF 驱动”

这个文件就要被重新设计，或者干脆被替换掉。

#### `local_planner/scripts/controller.py`

这个文件除了发 `/cmd_vel`，还会定时从 TF 读取 `world -> base_link`，然后发布 `/curr_state`。  
所以它实际上承担了“规划器看到的机器人状态从哪里来”这个职责。

如果你要改：

- 机器人状态发布频率
- 状态格式
- yaw 计算方式
- 由 TF 切换为订阅 odom / EKF 输出

这里也要一起改。

#### `jackal/jackal_control/config/robot_localization.yaml`

如果你要用 `robot_localization` 做真实一些的位姿融合，这个文件是核心。  
它定义：

- 订阅哪些传感器
- 哪些状态量参与融合
- 世界坐标系与机体系关系

#### `jackal/jackal_control/launch/control.launch`

这里通过 `enable_ekf` 参数决定是否启用 EKF。  
当前 `sim_environment.launch` 里把它设成了 `false`，所以默认并没有走 EKF 融合链。

## 5. 避障算法优化时，应该改哪些文件

避障层的核心问题包括：

- 参考轨迹怎么选
- 障碍物约束怎么建
- 安全距离怎么定
- 优化目标怎么配
- 失败回退怎么做
- 控制怎样送到底盘

### 5.1 当前默认运行的规划器

#### `local_planner/scripts/my_local_planner.py`

这个文件是当前默认运行的规划器实现。  
它直接利用预测椭圆的长短轴和朝向构造安全函数，并在原始椭圆 DCBF-MPC 思路上加入了你当前维护的参数化入口。

如果你要做：

- 椭圆级别障碍物建模
- 椭圆 CBF 约束改进
- 不确定性椭圆膨胀
- 与论文公式严格对照

当前工作区默认跑的就是这套 `my_local_planner`，所以避障算法的第一入口就是它。

### 5.2 当前该改哪个规划器

可以这样判断：

- 想优化当前默认运行效果，改 `my_local_planner.py`
- 想改启动方式，改 `local_planner/launch/local_planner.launch`
- 想改执行方式和状态来源，改 `local_planner/scripts/controller.py`

### 5.3 控制桥

#### `local_planner/scripts/controller.py`

这个文件负责：

- 从 `/local_plan` 读控制序列
- 取第一步控制量
- 发布 `/cmd_vel`
- 发布 `/curr_state`

它不是 MPC 求解器，但对最终控制效果有直接影响。

特别要注意：

- 规划器时域 `N = 25`
- `controller.py` 里本地缓存 `N = 10`

虽然它每次只执行第一步控制，但如果你要系统性重构控制序列执行方式，这个文件一定要一起看。

### 5.4 参考路径

#### `global_path_publisher/src/global_path_publisher.cpp`

当前全局路径非常简单：就是起点到终点的一条直线。  
如果你觉得局部规划器很容易，但只是因为全局参考太简单，那么这里是必须升级的。

可以从这里开始做：

- 曲线路径
- 栅格 / A* / Hybrid A*
- 多段路径
- 动态重规划

## 6. 一些常见优化需求，对应改哪些文件

### 6.1 我想让仿真更真实

优先顺序：

1. `scene/worlds/world.world`
2. `scene/launch/movetest_node.launch`
3. `scene/src/movetest.cpp`
4. `jackal/jackal_description/urdf/jackal.urdf.xacro`
5. `jackal/jackal_description/urdf/jackal.gazebo`
6. `jackal/jackal_control/config/control.yaml`

### 6.2 我想减少障碍物误检 / 漏检

优先顺序：

1. `local_map/launch/for_simulation.launch`
2. `local_map/src/local_map_pub.cpp`
3. `local_map/include/DBSCAN.hpp`
4. `local_map/include/ellipse.hpp`

### 6.3 我想让障碍物预测更准

优先顺序：

1. `obs_param/src/obs_kf.cpp`
2. `obs_param/include/kalman.h`
3. `obs_param/src/kalman.cpp`
4. `local_map/include/KM.hpp`

注意：如果 ID 跟踪先错了，后面的预测也会跟着错。

### 6.4 我想让避障更平滑、更安全或者更激进

优先顺序：

1. `local_planner/config/my_planner_params.yaml`
2. `global_path_publisher/src/global_path_publisher.cpp`
3. `local_planner/scripts/controller.py`
4. `jackal/jackal_control/config/control.yaml`

### 6.5 我想研究更真实的机器人定位与控制闭环

优先顺序：

1. `jackal/jackal_control/config/robot_localization.yaml`
2. `jackal/jackal_control/launch/control.launch`
3. `scene/src/getrobot_pose.cpp`
4. `local_planner/scripts/controller.py`

## 7. 建议你实际修改时采用的顺序

最稳妥的顺序通常是：

1. 先确定问题属于哪一层。
2. 先改 launch 和参数文件，验证“方向对不对”。
3. 再改主逻辑文件。
4. 最后再决定要不要动底层公用头文件、滤波器实现、URDF 和控制器。

一个很实用的原则是：

- 先改参数文件，后改算法逻辑
- 先改单层，后改跨层
- 先改输入稳定性，后改规划器复杂度

因为很多“避障失败”，根因其实并不在 MPC，而是在：

- 点云输入质量差
- 障碍物聚类不稳
- label 关联错
- 障碍物预测过度膨胀
- 底盘执行边界与规划边界不一致

## 8. 当前仓库里值得特别记住的结论

如果你只记住下面 4 句话，已经足够定位大部分修改入口：

1. 场景与动态障碍物看 `scene`，机器人和传感器看 `jackal/*`。
2. 感知主逻辑基本都在 `local_map/src/local_map_pub.cpp`。
3. 障碍物预测主逻辑基本都在 `obs_param/src/obs_kf.cpp`。
4. 当前默认运行的局部规划器就是 `my_local_planner.py`。
