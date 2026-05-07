#!/usr/bin/env python3

import threading

import casadi as ca
import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, ColorRGBA, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


def distance_global(c1, c2):
    return np.sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]))


class MyLocalPlanner:
    def __init__(self):
        # 重规划周期，决定局部规划器多久重新求解一次 MPC。
        self.replan_period = rospy.get_param("/local_planner/replan_period", 0.05)

        # ===== 1. 规划器基本参数 =====
        # N：预测时域长度
        # dt：离散时间步长
        # gamma_k：离散 CBF 收缩系数
        # v / omega：控制边界
        self.N = rospy.get_param("/my_local_planner/horizon", 25)
        self.dt = rospy.get_param("/my_local_planner/dt", 0.1)
        self.gamma_k = rospy.get_param("/my_local_planner/gamma_k", 0.3)
        self.v_max = rospy.get_param("/my_local_planner/v_max", 1.0)
        self.v_min = rospy.get_param("/my_local_planner/v_min", 0.0)
        self.omega_max = rospy.get_param("/my_local_planner/omega_max", 1.2)
        self.robot_radius = rospy.get_param("/my_local_planner/robot_radius", 0.35)
        self.safety_margin = rospy.get_param("/my_local_planner/safety_margin", 0.2)
        self.enable_cbf = rospy.get_param("/my_local_planner/enable_cbf", True)
        self.use_slack = rospy.get_param("/my_local_planner/use_slack", True)
        self.slack_weight = rospy.get_param("/my_local_planner/slack_weight", 500.0)
        self.goal_tolerance = rospy.get_param("/my_local_planner/goal_tolerance", 0.15)
        self.publish_vehicle_box = rospy.get_param("/my_local_planner/publish_vehicle_box", True)
        self.enable_visibility_constraint = rospy.get_param("/my_local_planner/enable_visibility_constraint", False)
        self.visibility_fov_deg = rospy.get_param("/my_local_planner/visibility_fov_deg", 70.0)
        self.visibility_ref_step = max(1, int(rospy.get_param("/my_local_planner/visibility_ref_step", 6)))
        self.visibility_margin = rospy.get_param("/my_local_planner/visibility_margin", 0.02)
        self.visibility_use_slack = rospy.get_param("/my_local_planner/visibility_use_slack", True)
        self.visibility_slack_weight = rospy.get_param("/my_local_planner/visibility_slack_weight", 300.0)
        self.visibility_cos_half_fov = np.cos(np.deg2rad(0.5 * self.visibility_fov_deg))
        self.collision_recovery_enable = rospy.get_param("/my_local_planner/collision_recovery_enable", True)
        self.collision_recovery_allow_reverse = rospy.get_param("/my_local_planner/collision_recovery_allow_reverse", True)
        self.collision_recovery_reverse_speed = rospy.get_param("/my_local_planner/collision_recovery_reverse_speed", 0.15)
        self.collision_recovery_turn_rate = rospy.get_param("/my_local_planner/collision_recovery_turn_rate", 0.6)
        self.collision_recovery_release_visibility = rospy.get_param(
            "/my_local_planner/collision_recovery_release_visibility", True
        )

        # ===== 2. 平滑性与终端收敛参数 =====
        # 这几项是本次重点新增的“更平滑”调节参数：
        # 1. control_weight_*：直接惩罚控制量幅值，避免速度/角速度过大
        # 2. delta_weight_*：惩罚相邻时刻控制变化，抑制“猛打一把方向”
        # 3. terminal_slack_weight：终端安全软约束惩罚，避免终端硬约束导致急转
        self.control_weight_v = rospy.get_param("/my_local_planner/control_weight_v", 0.15)
        self.control_weight_omega = rospy.get_param("/my_local_planner/control_weight_omega", 0.35)
        self.delta_weight_v = rospy.get_param("/my_local_planner/delta_weight_v", 0.4)
        self.delta_weight_omega = rospy.get_param("/my_local_planner/delta_weight_omega", 2.5)
        self.terminal_slack_weight = rospy.get_param("/my_local_planner/terminal_slack_weight", 800.0)
        self.cruise_v_min = rospy.get_param("/my_local_planner/cruise_v_min", 0.15)
        self.near_goal_dist = rospy.get_param("/my_local_planner/near_goal_dist", 0.8)
        self.obstacle_padding = rospy.get_param("/my_local_planner/obstacle_padding", 0.22)
        self.obstacle_relevance_dist = rospy.get_param("/my_local_planner/obstacle_relevance_dist", 2.5)
        self.dynamic_speed_threshold = rospy.get_param("/my_local_planner/dynamic_speed_threshold", 0.12)
        self.static_obstacle_padding = rospy.get_param("/my_local_planner/static_obstacle_padding", 0.14)
        self.dynamic_obstacle_padding = rospy.get_param("/my_local_planner/dynamic_obstacle_padding", 0.32)
        self.static_gamma_k = rospy.get_param("/my_local_planner/static_gamma_k", 0.35)
        self.dynamic_gamma_k = rospy.get_param("/my_local_planner/dynamic_gamma_k", 0.12)
        self.static_relevance_dist = rospy.get_param("/my_local_planner/static_relevance_dist", 2.4)
        self.dynamic_relevance_dist = rospy.get_param("/my_local_planner/dynamic_relevance_dist", 4.0)
        self.static_slack_weight_scale = rospy.get_param("/my_local_planner/static_slack_weight_scale", 0.5)
        self.dynamic_slack_weight_scale = rospy.get_param("/my_local_planner/dynamic_slack_weight_scale", 1.6)

        self.z = 0.0
        # goal_state 存储从全局路径上截取出的局部参考轨迹，每一行是 [x, y, yaw]。
        self.goal_state = np.zeros((self.N, 3))

        # 当前机器人状态 / 全局路径 / 障碍物预测，由各自回调函数维护。
        self.curr_state = None
        self.global_path = None
        self.ob = []

        # 保存上一轮优化解，用于：
        # 1. warm start，加快 IPOPT 收敛
        # 2. 求解失败时回退，避免控制量直接断掉
        self.last_input = np.zeros((self.N, 2))
        self.last_state = np.zeros((self.N + 1, 3))
        self.last_state_valid = False

        self.curr_pose_lock = threading.Lock()
        self.global_path_lock = threading.Lock()
        self.obstacle_lock = threading.Lock()

        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)
        self.__sub_curr_state = rospy.Subscriber("/curr_state", Float32MultiArray, self.__curr_pose_cb, queue_size=10)
        self.__sub_obs = rospy.Subscriber("/obs_predict_pub", Float32MultiArray, self.__obs_cb, queue_size=10)
        self.__sub_goal = rospy.Subscriber("/global_path", Path, self.__global_path_cb, queue_size=25)

        self.__pub_local_path_vis = rospy.Publisher("/pub_path_vis", Marker, queue_size=10)
        self.__pub_local_path = rospy.Publisher("/local_path", Path, queue_size=10)
        self.__pub_local_plan = rospy.Publisher("/local_plan", Float32MultiArray, queue_size=10)
        self.__pub_start = rospy.Publisher("/cmd_move", Bool, queue_size=10)
        self.__pub_visibility_refs_vis = rospy.Publisher("/visibility_refs_vis", MarkerArray, queue_size=10)

    def __replan_cb(self, _event):
        # 每次定时器触发时：
        # 1. 从全局路径中截取当前局部参考
        # 2. 为参考点补上航向角
        # 3. 求解 MPC-DCBF
        # 4. 发布局部轨迹与控制序列
        if not self.choose_goal_state():
            return

        for i in range(self.N - 1):
            y_diff = self.goal_state[i + 1, 1] - self.goal_state[i, 1]
            x_diff = self.goal_state[i + 1, 0] - self.goal_state[i, 0]
            if x_diff != 0 or y_diff != 0:
                self.goal_state[i, 2] = np.arctan2(y_diff, x_diff)
            elif i != 0:
                self.goal_state[i, 2] = self.goal_state[i - 1, 2]
            else:
                self.goal_state[i, 2] = 0.0
        self.goal_state[-1, 2] = self.goal_state[-2, 2]

        states_sol, input_sol = self.solve_mpc_cbf()

        cmd_move = Bool()
        cmd_move.data = distance_global(self.curr_state, self.global_path[-1]) > self.goal_tolerance
        self.__pub_start.publish(cmd_move)
        self.__publish_local_plan(input_sol, states_sol)

    def __curr_pose_cb(self, data):
        # /curr_state = [x, y, yaw]
        with self.curr_pose_lock:
            self.curr_state = np.array([data.data[0], data.data[1], data.data[2]], dtype=float)

    def __obs_cb(self, data):
        # /obs_predict_pub 中每 5 个数表示一个障碍物预测：
        # [cx, cy, a, b, theta]
        with self.obstacle_lock:
            self.ob = []
            size = int(len(data.data) / 5)
            for i in range(size):
                obstacle = np.array(data.data[5 * i:5 * i + 5], dtype=float)
                if np.all(np.isfinite(obstacle)):
                    self.ob.append(obstacle)

    def __global_path_cb(self, path):
        with self.global_path_lock:
            size = len(path.poses)
            if size > 0:
                self.global_path = np.zeros((size, 3))
                for i in range(size):
                    self.global_path[i, 0] = path.poses[i].pose.position.x
                    self.global_path[i, 1] = path.poses[i].pose.position.y

    def __publish_local_plan(self, input_sol, state_sol):
        # 将优化结果同时打包成：
        # 1. /local_path：预测轨迹
        # 2. /local_plan：控制序列 [v0, w0, v1, w1, ...]
        # 3. /pub_path_vis：RViz 可视化
        local_path = Path()
        local_plan = Float32MultiArray()
        local_path_vis = Marker()
        local_path_vis.header.stamp = rospy.Time.now()
        local_path_vis.header.frame_id = "world"
        local_path_vis.type = Marker.LINE_LIST
        local_path_vis.action = Marker.ADD
        local_path_vis.scale.x = 0.05
        local_path_vis.color.g = 1.0
        local_path_vis.color.b = 1.0
        local_path_vis.color.a = 1.0
        local_path_vis.pose.orientation.w = 1.0

        local_path.header.stamp = rospy.Time.now()
        local_path.header.frame_id = "world"

        for i in range(self.N):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"
            pose.pose.position.x = state_sol[i, 0]
            pose.pose.position.y = state_sol[i, 1]
            pose.pose.position.z = self.z
            pose.pose.orientation.w = 1.0
            local_path.poses.append(pose)

            local_plan.data.append(float(input_sol[i, 0]))
            local_plan.data.append(float(input_sol[i, 1]))

            if self.publish_vehicle_box and i < self.N - 1:
                self._append_vehicle_box(local_path_vis, state_sol, i)

        self.__pub_local_path_vis.publish(local_path_vis)
        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)
        self._publish_visibility_markers(state_sol)

    def _append_vehicle_box(self, local_path_vis, state_sol, i):
        # 这部分只用于 RViz 中显示车体外框，不参与 MPC 求解。
        x_diff = state_sol[i + 1, 0] - state_sol[i, 0]
        y_diff = state_sol[i + 1, 1] - state_sol[i, 1]
        theta = np.arctan2(y_diff, x_diff) if (x_diff != 0 or y_diff != 0) else state_sol[i, 2]
        pt_x = state_sol[i, 0]
        pt_y = state_sol[i, 1]

        color = ColorRGBA(1.0, 0.82, 0.1, 1.0)
        w = 0.7
        l = 0.92

        corners = []
        for sx, sy in [(1, 1), (-1, 1), (-1, -1), (1, -1)]:
            p = Point()
            p.z = -0.01
            p.x = 0.5 * (sx * l * np.cos(theta) - sy * w * np.sin(theta)) + pt_x
            p.y = 0.5 * (sx * l * np.sin(theta) + sy * w * np.cos(theta)) + pt_y
            corners.append(p)

        edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
        for s, e in edges:
            local_path_vis.points.append(corners[s])
            local_path_vis.colors.append(color)
            local_path_vis.points.append(corners[e])
            local_path_vis.colors.append(color)

    def _visibility_score(self, state, ref_xy):
        rel_x = ref_xy[0] - state[0]
        rel_y = ref_xy[1] - state[1]
        dist = np.hypot(rel_x, rel_y)
        if dist < 1e-6:
            return 0.0
        forward = rel_x * np.cos(state[2]) + rel_y * np.sin(state[2])
        return forward - dist * self.visibility_cos_half_fov - self.visibility_margin

    def _publish_visibility_markers(self, state_sol):
        marker_array = MarkerArray()

        def add_delete_marker(marker_id):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = "world"
            marker.id = marker_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        if not self.enable_visibility_constraint:
            add_delete_marker(0)
            add_delete_marker(1)
            add_delete_marker(2)
            self.__pub_visibility_refs_vis.publish(marker_array)
            return

        vis_refs = self._build_visibility_refs()

        refs_marker = Marker()
        refs_marker.header.stamp = rospy.Time.now()
        refs_marker.header.frame_id = "world"
        refs_marker.ns = "visibility_refs"
        refs_marker.id = 0
        refs_marker.type = Marker.SPHERE_LIST
        refs_marker.action = Marker.ADD
        refs_marker.pose.orientation.w = 1.0
        refs_marker.scale.x = 0.16
        refs_marker.scale.y = 0.16
        refs_marker.scale.z = 0.16
        refs_marker.color.r = 0.10
        refs_marker.color.g = 0.85
        refs_marker.color.b = 0.95
        refs_marker.color.a = 0.9

        path_marker = Marker()
        path_marker.header = refs_marker.header
        path_marker.ns = "visibility_ref_path"
        path_marker.id = 1
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.05
        path_marker.color.r = 0.10
        path_marker.color.g = 0.85
        path_marker.color.b = 0.95
        path_marker.color.a = 0.6

        for ref_xy in vis_refs:
            pt = Point()
            pt.x = float(ref_xy[0])
            pt.y = float(ref_xy[1])
            pt.z = 0.06
            refs_marker.points.append(pt)
            path_marker.points.append(pt)

        active_marker = Marker()
        active_marker.header = refs_marker.header
        active_marker.ns = "active_visibility_ref"
        active_marker.id = 2
        active_marker.type = Marker.ARROW
        active_marker.action = Marker.ADD
        active_marker.pose.orientation.w = 1.0
        active_marker.scale.x = 0.07
        active_marker.scale.y = 0.14
        active_marker.scale.z = 0.18

        score = self._visibility_score(state_sol[0], vis_refs[0])
        if score >= 0.0:
            active_marker.color.r = 0.20
            active_marker.color.g = 0.95
            active_marker.color.b = 0.20
        else:
            active_marker.color.r = 0.95
            active_marker.color.g = 0.20
            active_marker.color.b = 0.20
        active_marker.color.a = 0.95

        start_pt = Point()
        start_pt.x = float(state_sol[0, 0])
        start_pt.y = float(state_sol[0, 1])
        start_pt.z = 0.10
        end_pt = Point()
        end_pt.x = float(vis_refs[0, 0])
        end_pt.y = float(vis_refs[0, 1])
        end_pt.z = 0.10
        active_marker.points = [start_pt, end_pt]

        marker_array.markers.extend([refs_marker, path_marker, active_marker])
        self.__pub_visibility_refs_vis.publish(marker_array)

    def choose_goal_state(self):
        # 从全局路径中找到距离当前机器人最近的点，并向前截取 N 个点。
        # 这一步把“全局导航任务”转换成“局部 MPC 参考轨迹”。
        with self.curr_pose_lock:
            curr_state = None if self.curr_state is None else self.curr_state.copy()
        with self.global_path_lock:
            global_path = None if self.global_path is None else self.global_path.copy()

        if global_path is None or curr_state is None:
            return False

        waypoint_num = global_path.shape[0]
        num = np.argmin(np.array([distance_global(curr_state, global_path[i]) for i in range(waypoint_num)]))
        for i in range(self.N):
            num_path = min(waypoint_num - 1, num + i)
            self.goal_state[i] = global_path[num_path]
        return True

    def _obstacle_groups(self):
        # /obs_predict_pub 的排列方式是：
        # [障碍物1的N步预测, 障碍物2的N步预测, ...]
        # 这里把一维列表重新整理成“按障碍物分组”的二维结构。
        if not self.ob:
            return []

        obs_array = np.asarray(self.ob, dtype=float)
        num_obs = max(1, obs_array.shape[0] // self.N)
        groups = []
        for j in range(num_obs):
            start_idx = j * self.N
            end_idx = start_idx + self.N
            if end_idx > obs_array.shape[0]:
                break
            group = obs_array[start_idx:end_idx]
            if group.shape[0] == self.N:
                groups.append(group)
        return groups

    def _estimate_obstacle_speed(self, group):
        # 根据预测序列首尾位移粗略估计障碍物速度，用于区分静态 / 动态障碍。
        if group.shape[0] < 2:
            return 0.0
        displacement = np.linalg.norm(group[-1, :2] - group[0, :2])
        duration = max((group.shape[0] - 1) * self.dt, 1e-3)
        return displacement / duration

    def _obstacle_profile(self, group):
        # 为不同障碍物分配不同的安全策略：
        # 1. 静态障碍：允许更贴近，减少绕大圈
        # 2. 动态障碍：更早纳入约束，安全边界更大，slack 惩罚更重
        speed = self._estimate_obstacle_speed(group)
        is_dynamic = speed > self.dynamic_speed_threshold
        return {
            "is_dynamic": is_dynamic,
            "padding": self.dynamic_obstacle_padding if is_dynamic else self.static_obstacle_padding,
            "gamma_k": self.dynamic_gamma_k if is_dynamic else self.static_gamma_k,
            "relevance_dist": self.dynamic_relevance_dist if is_dynamic else self.static_relevance_dist,
            "slack_weight_scale": self.dynamic_slack_weight_scale if is_dynamic else self.static_slack_weight_scale,
        }

    def _ellipse_distance(self, state_xy, obs_step, padding):
        # 椭圆障碍物安全函数：
        # 返回值 > 0 表示位于膨胀椭圆外部，< 0 表示进入危险区域。
        # 这里恢复为“椭圆几何”而不是外接圆，避免长条障碍把路径过度挤弯。
        c = np.cos(float(obs_step[4]))
        s = np.sin(float(obs_step[4]))
        a = max(float(obs_step[2]), 1e-3)
        b = max(float(obs_step[3]), 1e-3)
        dx = float(state_xy[0]) - float(obs_step[0])
        dy = float(state_xy[1]) - float(obs_step[1])

        quad = (
            (c * c / (a * a) + s * s / (b * b)) * dx * dx
            + (s * s / (a * a) + c * c / (b * b)) * dy * dy
            + 2.0 * c * s * (1.0 / (a * a) - 1.0 / (b * b)) * dx * dy
        )
        return b * (np.sqrt(max(quad, 0.0)) - 1.0) - padding

    def _is_obstacle_relevant(self, curr_state, group):
        # 并不是所有障碍物都该进入当前 MPC。
        # 这里只保留“离机器人不远、并且确实挡在当前参考线前方”的障碍物，
        # 这样能明显减少无关障碍把路径拉弯的问题。
        profile = self._obstacle_profile(group)
        first_obs = group[0]
        obs_center = first_obs[:2]

        if distance_global(curr_state, obs_center) > profile["relevance_dist"]:
            return False

        goal_vec = self.goal_state[-1, :2] - curr_state[:2]
        obs_vec = obs_center - curr_state[:2]
        if np.dot(goal_vec, obs_vec) <= 0.0:
            return False

        path_norm = np.linalg.norm(goal_vec)
        if path_norm < 1e-6:
            return True

        lateral_dist = np.abs(goal_vec[0] * obs_vec[1] - goal_vec[1] * obs_vec[0]) / path_norm
        inflated_half_width = max(float(first_obs[2]), float(first_obs[3])) + self.robot_radius + self.safety_margin
        if profile["is_dynamic"]:
            inflated_half_width += 0.35
        return lateral_dist <= inflated_half_width + 0.6

    def _build_fallback(self, curr_state):
        # 求解失败时的回退策略：
        # 1. 默认停车
        # 2. 如果上一轮有可用解，就把上一轮解整体向前平移一拍继续用
        state_res = np.tile(curr_state, (self.N + 1, 1))
        input_res = np.zeros((self.N, 2))

        if self.last_state_valid:
            input_res[:-1] = self.last_input[1:]
            state_res[:-1] = self.last_state[1:]
            state_res[-1] = state_res[-2]

        self.last_input = input_res.copy()
        self.last_state = state_res.copy()
        self.last_state_valid = True
        return state_res, input_res

    @staticmethod
    def _normalize_angle(angle):
        return np.arctan2(np.sin(angle), np.cos(angle))

    def _build_recovery_fallback(self, curr_state, obstacle_step):
        # 当前状态已经压进膨胀障碍物时，单纯停车通常会一直卡在碰撞状态里。
        # 这里给一个保守的“倒车 + 远离障碍物方向转向”的开环恢复动作。
        obstacle_vec = obstacle_step[:2] - curr_state[:2]
        rel_bearing = self._normalize_angle(np.arctan2(obstacle_vec[1], obstacle_vec[0]) - curr_state[2])
        turn_sign = -1.0 if rel_bearing > 0.0 else 1.0
        v_cmd = -abs(self.collision_recovery_reverse_speed) if self.collision_recovery_allow_reverse else 0.0
        omega_cmd = turn_sign * abs(self.collision_recovery_turn_rate)

        input_res = np.tile(np.array([v_cmd, omega_cmd], dtype=float), (self.N, 1))
        state_res = np.zeros((self.N + 1, 3), dtype=float)
        state_res[0] = curr_state.copy()
        for i in range(self.N):
            state_res[i + 1, 0] = state_res[i, 0] + self.dt * input_res[i, 0] * np.cos(state_res[i, 2])
            state_res[i + 1, 1] = state_res[i, 1] + self.dt * input_res[i, 0] * np.sin(state_res[i, 2])
            state_res[i + 1, 2] = self._normalize_angle(state_res[i, 2] + self.dt * input_res[i, 1])

        self.last_input = input_res.copy()
        self.last_state = state_res.copy()
        self.last_state_valid = True
        return state_res, input_res

    def _build_visibility_refs(self):
        # 几何视野约束不直接盯着最终终点，而是盯着“路径前方一点”：
        # 对预测第 i 步，要求更靠前的参考点仍落在车头视锥内。
        vis_refs = np.zeros((self.N, 2))
        for i in range(self.N):
            ref_idx = min(self.N - 1, i + self.visibility_ref_step)
            vis_refs[i] = self.goal_state[ref_idx, :2]
        return vis_refs

    def solve_mpc_cbf(self):
        # 这里是局部规划器核心：
        # 建立并求解“带离散 CBF 安全约束、带软约束、带平滑项”的 MPC。
        with self.curr_pose_lock:
            curr_state = self.curr_state.copy()
        with self.global_path_lock:
            global_path = self.global_path.copy()
        with self.obstacle_lock:
            obstacle_groups = self._obstacle_groups()
        active_obstacle_groups = [group for group in obstacle_groups if self._is_obstacle_relevant(curr_state, group)]
        obstacle_profiles = [self._obstacle_profile(group) for group in active_obstacle_groups]
        inside_indices = [
            j
            for j, group in enumerate(active_obstacle_groups)
            if self._ellipse_distance(curr_state[:2], group[0], obstacle_profiles[j]["padding"]) < 0.0
        ]
        inside_any_obstacle = len(inside_indices) > 0
        use_visibility_constraint = self.enable_visibility_constraint and not (
            inside_any_obstacle and self.collision_recovery_release_visibility
        )

        opti = ca.Opti()

        # ===== 1. 优化变量 =====
        # opt_states: 预测状态序列 X = [x_k, y_k, yaw_k]
        # opt_controls: 预测控制序列 U = [v_k, omega_k]
        opt_states = opti.variable(self.N + 1, 3)
        opt_controls = opti.variable(self.N, 2)
        slack = opti.variable(self.N, len(active_obstacle_groups)) if (self.use_slack and active_obstacle_groups) else None
        terminal_slack = opti.variable(len(active_obstacle_groups)) if (self.use_slack and active_obstacle_groups) else None
        vis_slack = opti.variable(self.N) if (use_visibility_constraint and self.visibility_use_slack) else None
        opt_x0 = opti.parameter(3)

        v = opt_controls[:, 0]
        omega = opt_controls[:, 1]

        def f(x_, u_):
            # 独轮车 / 单轨运动学模型。
            return ca.vertcat(u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1])

        def barrier(state_, obs_step, padding):
            # 椭圆障碍物的 CasADi 安全函数，和原版 `local_planner.py` 更接近。
            c = ca.cos(obs_step[4])
            s = ca.sin(obs_step[4])
            a = ca.fmax(obs_step[2], 1e-3)
            b = ca.fmax(obs_step[3], 1e-3)
            dx = state_[0] - float(obs_step[0])
            dy = state_[1] - float(obs_step[1])
            quad = (
                (c * c / (a * a) + s * s / (b * b)) * dx * dx
                + (s * s / (a * a) + c * c / (b * b)) * dy * dy
                + 2.0 * c * s * (1.0 / (a * a) - 1.0 / (b * b)) * dx * dy
            )
            return b * (ca.sqrt(quad) - 1.0) - padding

        def quadratic(x_, a_):
            return ca.mtimes([x_, a_, x_.T])

        def angle_error(angle_, ref_angle_):
            # 航向误差必须做角度归一化，否则在 -pi / pi 附近会出现“明明快对齐却被当成大误差”，
            # 典型表现就是无障碍时原地打转。
            return ca.atan2(ca.sin(angle_ - ref_angle_), ca.cos(angle_ - ref_angle_))

        def visibility_barrier(state_, ref_xy):
            # 第一版视野约束采用几何视锥形式：
            # 前方参考点在当前车头朝向上的投影，必须大于“视锥边界”对应的最小投影。
            rel_x = float(ref_xy[0]) - state_[0]
            rel_y = float(ref_xy[1]) - state_[1]
            dist = ca.sqrt(rel_x * rel_x + rel_y * rel_y + 1e-6)
            forward = rel_x * ca.cos(state_[2]) + rel_y * ca.sin(state_[2])
            return forward - dist * self.visibility_cos_half_fov - self.visibility_margin

        # ===== 2. 初始状态约束与控制边界 =====
        # 预测域起点必须等于当前机器人状态。
        opti.subject_to(opt_states[0, :] == opt_x0.T)
        # 远离终点时要求机器人保持向前推进，避免在无障碍场景下原地找角度；
        # 接近终点时再放松到 0，便于最终收敛。
        dist_to_goal = distance_global(curr_state, global_path[-1, :2])
        if inside_any_obstacle and self.collision_recovery_enable and self.collision_recovery_allow_reverse:
            effective_v_min = min(self.v_min, -abs(self.collision_recovery_reverse_speed))
        else:
            effective_v_min = self.cruise_v_min if dist_to_goal > self.near_goal_dist else self.v_min
        opti.subject_to(opti.bounded(effective_v_min, v, self.v_max))
        opti.subject_to(opti.bounded(-self.omega_max, omega, self.omega_max))

        if slack is not None:
            for i in range(self.N):
                for j in range(len(active_obstacle_groups)):
                    opti.subject_to(slack[i, j] >= 0)
            for j in range(len(active_obstacle_groups)):
                opti.subject_to(terminal_slack[j] >= 0)
        if vis_slack is not None:
            for i in range(self.N):
                opti.subject_to(vis_slack[i] >= 0)

        # ===== 3. 系统动力学离散约束 =====
        # 将机器人运动学在整个预测时域内展开。
        for i in range(self.N):
            x_next = opt_states[i, :] + self.dt * f(opt_states[i, :], opt_controls[i, :]).T
            opti.subject_to(opt_states[i + 1, :] == x_next)

        # ===== 4. 离散 CBF 安全约束 =====
        # 对每个障碍物、每个预测步施加：
        #   h_{k+1} + slack_k >= (1 - gamma_k) h_k
        # 这样既能保证安全集前向不变性，又能在极端情况下通过 slack 防止直接不可行。
        if self.enable_cbf:
            for j, group in enumerate(active_obstacle_groups):
                profile = obstacle_profiles[j]
                h0 = self._ellipse_distance(curr_state[:2], group[0], profile["padding"])
                if h0 < 0:
                    rospy.logwarn_throttle(1.0, "Robot starts inside ellipse safety set of obstacle %d", j)

                for i in range(self.N - 1):
                    hk = barrier(opt_states[i, :], group[i], profile["padding"])
                    next_obs = group[i + 1]
                    hk_next = barrier(opt_states[i + 1, :], next_obs, profile["padding"])
                    rhs = (1.0 - profile["gamma_k"]) * hk
                    
                    # 引入离散松弛 CBF 约束
                    if slack is not None:
                        opti.subject_to(hk_next + slack[i, j] >= rhs)
                    else:
                        opti.subject_to(hk_next >= rhs)

                # 终端安全约束采用“软约束”而不是纯硬约束：
                # 这样可以显著缓解横向挡路障碍物场景下的急转问题。
                terminal_obs = group[-1]
                h_terminal = barrier(opt_states[self.N, :], terminal_obs, profile["padding"])
                if terminal_slack is not None:
                    opti.subject_to(h_terminal + terminal_slack[j] >= 0)
                else:
                    opti.subject_to(h_terminal >= 0)

        # ===== 4.1 几何视野约束 =====
        # 对每个预测步，要求“路径前方 look-ahead 参考点”仍在机器人当前视锥内。
        # 这会直接约束 yaw 的演化，并在转角/盲区入口前促使机器人更早减速和摆正车头。
        if use_visibility_constraint:
            vis_refs = self._build_visibility_refs()
            for i in range(self.N):
                h_vis = visibility_barrier(opt_states[i + 1, :], vis_refs[i])
                if vis_slack is not None:
                    opti.subject_to(h_vis + vis_slack[i] >= 0)
                else:
                    opti.subject_to(h_vis >= 0)

        # ===== 5. 目标函数 =====
        # 这里重新拉回到更接近原版的结构：
        # 1. 主体仍然是“跟踪全局直线路径”
        # 2. 控制变化率只做弱惩罚，不再主导轨迹形状
        # 3. 安全相关的 slack 仍然保留，但只对真正相关的障碍起作用
        obj = 0
        r_mat = np.diag([self.control_weight_v, self.control_weight_omega])
        delta_r_mat = np.diag([self.delta_weight_v, self.delta_weight_omega])
        # CasADi 这里使用的是 1x2 行向量，因此上一轮控制也要保持相同形状，
        # 否则会出现 (1x2) - (2x1) 的维度不匹配。
        prev_u = self.last_input[[0], :] if self.last_state_valid else np.zeros((1, 2))

        for i in range(self.N):
            # 位置误差仍然是主导项，鼓励机器人尽量贴着全局直线走。
            state_err = opt_states[i, :] - self.goal_state[[i]]
            yaw_err = angle_error(opt_states[i, 2], self.goal_state[i, 2])
            q_mat = np.diag([1.0 + 0.05 * i, 1.0 + 0.05 * i, 0.02 + 0.005 * i])
            obj += 0.1 * quadratic(state_err, q_mat)
            obj += 0.05 * yaw_err * yaw_err
            obj += quadratic(opt_controls[i, :], r_mat)

            # 控制变化率惩罚是本次“更平滑版本”的关键：
            # 对第 0 步，约束其不要与上一轮首拍控制差太大；
            # 对后续步，约束相邻两步控制不要突变。
            if i == 0:
                delta_u = opt_controls[i, :] - prev_u
            else:
                delta_u = opt_controls[i, :] - opt_controls[i - 1, :]
            obj += 0.2 * quadratic(delta_u, delta_r_mat)

            if slack is not None:
                for j in range(slack.shape[1]):
                    # 近期 slack 惩罚更大，远期稍小。
                    dynamic_weight = self.slack_weight * obstacle_profiles[j]["slack_weight_scale"] * ((self.N - i) / self.N)
                    obj += dynamic_weight * slack[i, j] * slack[i, j]

        if terminal_slack is not None:
            for j in range(len(active_obstacle_groups)):
                obj += self.terminal_slack_weight * terminal_slack[j] * terminal_slack[j]
        if vis_slack is not None:
            for i in range(self.N):
                obj += self.visibility_slack_weight * ((self.N - i) / self.N) * vis_slack[i] * vis_slack[i]

        # 终端状态继续向参考线末端收敛。
        terminal_pos_err = opt_states[self.N, :2] - self.goal_state[[self.N - 1], :2]
        terminal_yaw_err = angle_error(opt_states[self.N, 2], self.goal_state[self.N - 1, 2])
        q_terminal_pos = np.diag([6.0, 6.0])
        obj += quadratic(terminal_pos_err, q_terminal_pos)
        obj += 0.1 * terminal_yaw_err * terminal_yaw_err
        opti.minimize(obj)

        # ===== 6. 求解器设置 =====
        opts_setting = {
            "ipopt.max_iter": 800,
            "ipopt.print_level": 0,
            "print_time": 0,
            "ipopt.acceptable_tol": 1e-3,
            "ipopt.acceptable_obj_change_tol": 1e-3,
        }
        opti.solver("ipopt", opts_setting)
        opti.set_value(opt_x0, curr_state)

        if self.last_state_valid:
            opti.set_initial(opt_controls, self.last_input)
            opti.set_initial(opt_states, self.last_state)
            if slack is not None:
                opti.set_initial(slack, 0)
                opti.set_initial(terminal_slack, 0)
            if vis_slack is not None:
                opti.set_initial(vis_slack, 0)

        try:
            sol = opti.solve()
            u_res = sol.value(opt_controls)
            state_res = sol.value(opt_states)

            # 求解成功时缓存本轮结果，供下一轮 warm start 和回退使用。
            self.last_input = np.array(u_res, dtype=float)
            self.last_state = np.array(state_res, dtype=float)
            self.last_state_valid = True
            return self.last_state, self.last_input
        except Exception:
            # 当前回退策略保持为“沿用上一轮解并平移”，优先保证连续性。
            rospy.logerr("My planner: Infeasible Solution")
            if inside_any_obstacle and self.collision_recovery_enable:
                recovery_obs = active_obstacle_groups[inside_indices[0]][0]
                return self._build_recovery_fallback(curr_state, recovery_obs)
            return self._build_fallback(curr_state)


if __name__ == "__main__":
    rospy.init_node("my_local_planner")
    planner = MyLocalPlanner()
    rospy.spin()
