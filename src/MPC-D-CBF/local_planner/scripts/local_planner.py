#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32MultiArray, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
import threading
import numpy as np
from visualization_msgs.msg import Marker
import casadi as ca


def distance_global(c1, c2):
    return np.sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]))


class Local_Planner():
    def __init__(self):
        # 重规划周期，决定局部规划器多久重新求解一次 MPC。
        self.replan_period = rospy.get_param('/local_planner/replan_period', 0.05)

        self.z = 0
        # 预测时域长度 N：后续所有状态、控制、障碍物预测都按这个长度展开。
        self.N = 25
        # goal_state 存储从全局路径上截取出的局部参考轨迹，每一行是 [x, y, yaw]。
        self.goal_state = np.zeros([self.N, 3])

        # 当前机器人状态 / 全局参考路径，由回调函数实时刷新。
        self.curr_state = None
        self.global_path = None

        # 求解失败时回退到上一轮解，避免控制直接中断。
        self.last_input = np.zeros((self.N, 2))
        self.last_state = np.zeros((self.N + 1, 3))
        self.mpc_success = False

        self.curr_pose_lock = threading.Lock()
        self.global_path_lock = threading.Lock()
        self.obstacle_lock = threading.Lock()

        # 未来障碍物预测序列。obs_kf 发布格式为 [cx, cy, a, b, theta] * (障碍物数 * N)。
        self.ob = []

        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)

        self.__sub_curr_state = rospy.Subscriber('/curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=10)

        self.__sub_obs = rospy.Subscriber('/obs_predict_pub', Float32MultiArray, self.__obs_cb, queue_size=10)

        self.__sub_goal = rospy.Subscriber('/global_path', Path, self.__global_path_cb, queue_size=25)

        self.__pub_local_path_vis = rospy.Publisher('/pub_path_vis', Marker, queue_size=10)

        self.__pub_local_path = rospy.Publisher('/local_path', Path, queue_size=10)

        self.__pub_local_plan = rospy.Publisher('/local_plan', Float32MultiArray, queue_size=10)

        self.__pub_start = rospy.Publisher('/cmd_move', Bool, queue_size=10)

    def __replan_cb(self, event):
        if self.choose_goal_state():
            # 从局部参考路径点差分恢复参考航向角。
            for i in range(self.N - 1):
                y_diff = self.goal_state[i+1, 1]-self.goal_state[i, 1]
                x_diff = self.goal_state[i+1, 0]-self.goal_state[i, 0]
                if x_diff != 0 and y_diff != 0:
                    self.goal_state[i, 2] = np.arctan2(y_diff, x_diff)
                elif i != 0:
                    self.goal_state[i, 2] = self.goal_state[i-1, 2]
                else:
                    self.goal_state[i, 2] = 0

            self.goal_state[-1, 2] = self.goal_state[-2, 2]

            states_sol, input_sol = self.MPC_ellip()

            # /cmd_move 不是控制机器人，而是给动态障碍物节点的启动信号。
            cmd_move = Bool()
            cmd_move.data = distance_global(self.curr_state, self.global_path[-1]) > 0.1
            self.__pub_start.publish(cmd_move)
            self.__publish_local_plan(input_sol, states_sol)

    def __curr_pose_cb(self, data):
        # /curr_state = [x, y, yaw]
        with self.curr_pose_lock:
            self.curr_state = np.zeros(3)
            self.curr_state[0] = data.data[0]
            self.curr_state[1] = data.data[1]
            self.curr_state[2] = data.data[2]

    def __obs_cb(self, data):
        # /obs_predict_pub 中每 5 个数对应一个预测椭圆 [cx, cy, a, b, theta]。
        with self.obstacle_lock:
            self.ob = []
            size = int(len(data.data) / 5)
            for i in range(size):
                self.ob.append(np.array(data.data[5*i:5*i+5]))

    def __global_path_cb(self, path):
        # /global_path 是全局参考路径，规划器每次只截取前方 N 个点作为局部参考。
        with self.global_path_lock:
            size = len(path.poses)
            if size > 0:
                self.global_path = np.zeros([size, 3])
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
        local_path_vis.type = Marker.LINE_LIST
        local_path_vis.scale.x = 0.05
        local_path_vis.color.g = local_path_vis.color.b = local_path_vis.color.a = 1.0

        local_path_vis.header.stamp = rospy.Time.now()
        local_path.header.stamp = rospy.Time.now()

        local_path.header.frame_id = "world"
        local_path_vis.header.frame_id = "world"

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.pose.position.x = state_sol[i, 0]
            this_pose_stamped.pose.position.y = state_sol[i, 1]
            this_pose_stamped.pose.position.z = self.z
            this_pose_stamped.pose.orientation.x = 0
            this_pose_stamped.pose.orientation.y = 0
            this_pose_stamped.pose.orientation.z = 0
            this_pose_stamped.pose.orientation.w = 1
            this_pose_stamped.header.seq = i
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id = "world"
            local_path.poses.append(this_pose_stamped)
            for j in range(2):
                if len(input_sol[i]) != 0:
                    local_plan.data.append(input_sol[i][j])

            pt = Point()
            pt.x = state_sol[i, 0]
            pt.y = state_sol[i, 1]
            pt.z = self.z

            # 下面这段是 RViz 中小车外框可视化，不参与 MPC 求解本身。
            color = ColorRGBA()
            color.r = 1
            color.g = 0.82
            color.b = 0.1
            color.a = 1

            p1 = Point()
            p2 = Point()
            p3 = Point()
            p4 = Point()
            if i < self.N-1:
                x_diff = state_sol[i+1, 0]-state_sol[i, 0]
                y_diff = state_sol[i+1, 1]-state_sol[i, 1]
                if x_diff != 0 and y_diff != 0:
                    theta = np.arctan2(y_diff, x_diff)
                else:
                    theta = 0
                w = 0.7
                l = 0.92
                p1.z = pt.z-0.01
                p1.x = 0.5*(l*np.cos(theta)-w*np.sin(theta)) + pt.x
                p1.y = 0.5*(l*np.sin(theta)+w*np.cos(theta)) + pt.y
                p2.z = pt.z-0.01
                p2.x = 0.5*(-l*np.cos(theta)-w*np.sin(theta)) + pt.x
                p2.y = 0.5*(-l*np.sin(theta)+w*np.cos(theta)) + pt.y
                p3.z = pt.z-0.01
                p3.x = 0.5*(-l*np.cos(theta)+w*np.sin(theta)) + pt.x
                p3.y = 0.5*(-l*np.sin(theta)-w*np.cos(theta)) + pt.y
                p4.z = pt.z-0.01
                p4.x = 0.5*(l*np.cos(theta)+w*np.sin(theta)) + pt.x
                p4.y = 0.5*(l*np.sin(theta)-w*np.cos(theta)) + pt.y

                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p2)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p3)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p4)
                local_path_vis.colors.append(color)
                local_path_vis.points.append(p1)
                local_path_vis.colors.append(color)
                local_path_vis.pose.orientation.x = 0
                local_path_vis.pose.orientation.y = 0
                local_path_vis.pose.orientation.z = 0
                local_path_vis.pose.orientation.w = 1

        self.__pub_local_path_vis.publish(local_path_vis)
        self.__pub_local_path.publish(local_path)
        self.__pub_local_plan.publish(local_plan)

    def choose_goal_state(self):
        # 从全局路径中找到距离当前机器人最近的路径点，并向前截取 N 个点。
        # 这一步相当于把“全局导航任务”转成“局部 MPC 参考轨迹”。
        with self.curr_pose_lock:
            with self.global_path_lock:
                if self.global_path is None or self.curr_state is None:
                    return False

                waypoint_num = self.global_path.shape[0]
                num = np.argmin(np.array([distance_global(self.curr_state, self.global_path[i]) for i in range(waypoint_num)]))

                scale = 1
                num_list = []
                for i in range(self.N):
                    num_path = min(waypoint_num - 1, num + i * scale)
                    num_list.append(num_path)

                for k in range(self.N):
                    self.goal_state[k] = self.global_path[num_list[k]]

        return True

    def MPC_ellip(self):
        # 这里是整个局部规划器的核心：建立并求解“带椭圆障碍物 DCBF 约束的 MPC”。
        with self.curr_pose_lock, self.global_path_lock, self.obstacle_lock:

        opti = ca.Opti()
        # ===== 1. 基本参数 =====
        # T：离散时间步长
        # gamma_k：离散 CBF 约束松紧参数
        # v / omega 上下界：控制约束
        T = 0.1
        #gamma_k = 0.15
        gamma_k = 0.4

        v_max = 1.2
        v_min = 0.1
        omega_max = 1.2

        opt_x0 = opti.parameter(3)

        # ===== 2. 优化变量 =====
        # opt_states: 预测状态序列 X = [x_k, y_k, yaw_k]
        # opt_controls: 预测控制序列 U = [v_k, omega_k]
        opt_states = opti.variable(self.N + 1, 3)
        opt_controls = opti.variable(self.N, 2)
        v = opt_controls[:, 0]
        omega = opt_controls[:, 1]

        # 独轮车/单轨运动学模型。
        def f(x_, u_): return ca.vertcat(*[u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1]])

        def exceed_ob(ob_):
            # 判断某个障碍物是否已经处在目标点后方。
            # 如果已经不构成当前前向通行约束，就不再为它添加 DCBF 约束，减少保守性。
            l_long_axis = ob_[2]
            l_short_axis = ob_[3]
            long_axis = np.array([np.cos(ob_[4]) * l_long_axis, np.sin(ob_[4]) * l_long_axis])

            ob_vec = np.array([ob_[0], ob_[1]])
            center_vec = self.goal_state[self.N-1, :2] - ob_vec
            dist_center = np.linalg.norm(center_vec)
            cos_ = np.dot(center_vec, long_axis.T) / (dist_center * l_long_axis)

            if np.abs(cos_) > 0.1:
                tan_square = 1 / (cos_ ** 2) - 1
                d = np.sqrt((l_long_axis ** 2 * l_short_axis ** 2 * (1 + tan_square) / (
                    l_short_axis ** 2 + l_long_axis ** 2 * tan_square)))
            else:
                d = l_short_axis

            cross_pt = ob_vec + d * center_vec / dist_center

            vec1 = self.goal_state[self.N-1, :2] - cross_pt
            vec2 = self.curr_state[:2] - cross_pt
            theta = np.dot(vec1.T, vec2)

            return theta > 0

        def h(curpos_, ob_):
            # ===== 3. 椭圆障碍物安全函数 h(x) =====
            # h > 0: 安全
            # h = 0: 位于安全边界
            # h < 0: 进入危险区域
            # safe_dist 是在椭圆几何边界之外额外膨胀的安全距离。
            #safe_dist = 1.3 # for scout
            safe_dist = 0.3 # for jackal

            c = ca.cos(ob_[4])
            s = ca.sin(ob_[4])
            a = ca.MX([ob_[2]])
            b = ca.MX([ob_[3]])

            ob_vec = ca.MX([ob_[0], ob_[1]])
            center_vec = curpos_[:2] - ob_vec.T

            dist = b * (ca.sqrt((c ** 2 / a ** 2 + s ** 2 / b ** 2) * center_vec[0] ** 2 + (s ** 2 / a ** 2 + c ** 2 / b ** 2) *
                                center_vec[1] ** 2 + 2 * c * s * (1 / a ** 2 - 1 / b ** 2) * center_vec[0] * center_vec[1]) - 1) - safe_dist
            
            return dist

        def quadratic(x, A):
            return ca.mtimes([x, A, x.T])

        # ===== 4. 初始状态约束 =====
        # MPC 预测的第一个状态必须等于当前机器人状态。
        opti.subject_to(opt_states[0, :] == opt_x0.T)

        # ===== 5. 控制边界约束 =====
        # 远离终点时要求前进速度非负；接近终点时允许轻微倒车，便于收敛到目标位姿。
        if(distance_global(self.curr_state, self.global_path[-1, :2]) > 1):
            opti.subject_to(opti.bounded(v_min, v, v_max))
        else:
            opti.subject_to(opti.bounded(-v_min, v, v_max))

        opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

        # ===== 6. 运动学约束 =====
        # 将系统模型离散化到整个预测域内，保证每一步状态演化都满足机器人运动学。
        for i in range(self.N):
            x_next = opt_states[i, :] + T * f(opt_states[i, :], opt_controls[i, :]).T
            opti.subject_to(opt_states[i + 1, :] == x_next)

        # /obs_predict_pub 的组织方式是：
        # 每个障碍物给出 N 步预测，每步一个椭圆，因此障碍物总数 = len(self.ob) / N。
        num_obs = int(len(self.ob)/self.N)

        # ===== 7. 离散 DCBF 约束 =====
        # 对每个障碍物、每个预测步施加：
        #   h_{k+1} >= (1 - gamma_k) * h_k
        # 这保证系统在离散时间下保持安全集前向不变性。
        for j in range(num_obs):
            if not exceed_ob(self.ob[25*j]):
                for i in range(self.N-1):
                    opti.subject_to(h(opt_states[i + 1, :], self.ob[j * 25 + i + 1]) >=
                                    (1 - gamma_k) * h(opt_states[i, :], self.ob[j * 25 + i]))

        # ===== 8. 目标函数 =====
        # 由两部分组成：
        # 1. 状态跟踪误差：逼近局部参考轨迹 goal_state
        # 2. 控制代价：抑制过大的线速度/角速度
        # 最后再额外加一个终端状态代价，增强收敛性。
        obj = 0
        R = np.diag([0.1, 0.02])
        A = np.diag([0.1, 0.02])
        for i in range(self.N):
            Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.02+0.005*i])
            if i < self.N-1:
                obj += 0.1 * quadratic(opt_states[i, :] - self.goal_state[[i]], Q) + quadratic(opt_controls[i, :], R)
            else:
                obj += 0.1 * quadratic(opt_states[i, :] - self.goal_state[[i]], Q)
        Q = np.diag([1.0,1.0, 0.02])*5
        obj += quadratic(opt_states[self.N-1, :] - self.goal_state[[self.N-1]], Q)

        # ===== 9. 求解器设置 =====
        opti.minimize(obj)
        opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-3,
                        'ipopt.acceptable_obj_change_tol': 1e-3}
        opti.solver('ipopt', opts_setting)
        opti.set_value(opt_x0, self.curr_state)

        try:
            sol = opti.solve()
            u_res = sol.value(opt_controls)
            state_res = sol.value(opt_states)

            # 求解成功时缓存本轮解，供下一轮 warm start / fallback 使用。
            self.last_input = u_res
            self.last_state = state_res
            self.mpc_success = True

        except:
            # ===== 10. 求解失败回退 =====
            # 当前版本的策略比较简单：如果失败，沿用上一轮解，并将控制序列整体向前平移。
            # 这也是你后续很值得改进的一块，例如：
            # - 加 slack 变量
            # - 失败时主动停车
            # - 切换备份行为策略
            rospy.logerr("Infeasible Solution")

            if self.mpc_success:
                self.mpc_success = False
            else:
                for i in range(self.N-1):
                    self.last_input[i] = self.last_input[i+1]
                    self.last_state[i] = self.last_state[i+1]
                    self.last_input[self.N-1] = np.zeros([1, 2])

            u_res = self.last_input
            state_res = self.last_state
        return state_res, u_res


if __name__ == '__main__':
    rospy.init_node("phri_planner")
    phri_planner = Local_Planner()

    rospy.spin()
