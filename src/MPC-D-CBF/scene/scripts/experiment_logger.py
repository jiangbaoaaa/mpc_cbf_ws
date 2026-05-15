#!/usr/bin/env python3

import csv
import math
import os
from datetime import datetime

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32MultiArray


class ExperimentLogger:
    def __init__(self):
        self.label = rospy.get_param("~label", "experiment")
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/.ros/mpc_dcbf_logs"))
        self.goal_tolerance = float(rospy.get_param("~goal_tolerance", 0.15))

        os.makedirs(self.output_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, "summary.csv")
        self._ensure_csv_header()

        self.goal_xy = None
        self.curr_xy = None
        self.prev_xy = None
        self.cmd_move = False
        self.active = False
        self.run_index = 0

        self._reset_metrics()

        self.sub_curr = rospy.Subscriber("/curr_state", Float32MultiArray, self.curr_state_cb, queue_size=20)
        self.sub_goal = rospy.Subscriber("/global_path", Path, self.global_path_cb, queue_size=5)
        self.sub_cmd_move = rospy.Subscriber("/cmd_move", Bool, self.cmd_move_cb, queue_size=20)
        self.sub_local_plan = rospy.Subscriber("/local_plan", Float32MultiArray, self.local_plan_cb, queue_size=20)
        self.sub_static_risk = rospy.Subscriber(
            "/local_map_pub/static_risk_points", Float32MultiArray, self.static_risk_cb, queue_size=20
        )

        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("Experiment logger writing to %s", self.csv_path)

    def _ensure_csv_header(self):
        if os.path.exists(self.csv_path):
            return
        with open(self.csv_path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(
                [
                    "timestamp",
                    "label",
                    "run_index",
                    "status",
                    "duration_sec",
                    "path_length_m",
                    "avg_cmd_v",
                    "max_abs_cmd_omega",
                    "min_static_risk_dist_m",
                    "final_goal_dist_m",
                    "visibility_enabled",
                    "static_risk_enabled",
                    "path_reference_speed",
                    "visibility_fov_deg",
                ]
            )

    def _reset_metrics(self):
        self.start_time = None
        self.path_length = 0.0
        self.cmd_v_sum = 0.0
        self.cmd_v_count = 0
        self.max_abs_cmd_omega = 0.0
        self.min_static_risk_dist = float("inf")

    def _distance_to_goal(self):
        if self.goal_xy is None or self.curr_xy is None:
            return float("inf")
        dx = self.curr_xy[0] - self.goal_xy[0]
        dy = self.curr_xy[1] - self.goal_xy[1]
        return math.hypot(dx, dy)

    def _begin_run(self):
        self.active = True
        self.run_index += 1
        self.start_time = rospy.Time.now()
        self.path_length = 0.0
        self.cmd_v_sum = 0.0
        self.cmd_v_count = 0
        self.max_abs_cmd_omega = 0.0
        self.min_static_risk_dist = float("inf")
        self.prev_xy = None if self.curr_xy is None else tuple(self.curr_xy)
        rospy.loginfo("Experiment run %d started (%s)", self.run_index, self.label)

    def _finish_run(self, status):
        if not self.active or self.start_time is None:
            return

        end_time = rospy.Time.now()
        duration = max(0.0, (end_time - self.start_time).to_sec())
        final_goal_dist = self._distance_to_goal()
        avg_cmd_v = self.cmd_v_sum / self.cmd_v_count if self.cmd_v_count > 0 else 0.0
        min_static_risk_dist = self.min_static_risk_dist if math.isfinite(self.min_static_risk_dist) else float("nan")

        row = [
            datetime.now().isoformat(timespec="seconds"),
            self.label,
            self.run_index,
            status,
            round(duration, 4),
            round(self.path_length, 4),
            round(avg_cmd_v, 4),
            round(self.max_abs_cmd_omega, 4),
            round(min_static_risk_dist, 4) if math.isfinite(min_static_risk_dist) else "nan",
            round(final_goal_dist, 4),
            int(rospy.get_param("/my_local_planner/enable_visibility_constraint", False)),
            int(rospy.get_param("/my_local_planner/use_static_risk_points", False)),
            float(rospy.get_param("/my_local_planner/path_reference_speed", 0.0)),
            float(rospy.get_param("/my_local_planner/visibility_fov_deg", 0.0)),
        ]

        with open(self.csv_path, "a", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(row)

        rospy.loginfo(
            "Experiment run %d finished: status=%s duration=%.2fs path=%.2fm min_static_risk=%.2f",
            self.run_index,
            status,
            duration,
            self.path_length,
            min_static_risk_dist if math.isfinite(min_static_risk_dist) else float("nan"),
        )

        self.active = False
        self._reset_metrics()

    def curr_state_cb(self, msg):
        if len(msg.data) < 2:
            return
        self.curr_xy = (float(msg.data[0]), float(msg.data[1]))
        if self.active and self.prev_xy is not None:
            dx = self.curr_xy[0] - self.prev_xy[0]
            dy = self.curr_xy[1] - self.prev_xy[1]
            self.path_length += math.hypot(dx, dy)
        self.prev_xy = self.curr_xy

    def global_path_cb(self, msg):
        if not msg.poses:
            self.goal_xy = None
            return
        goal_pose = msg.poses[-1].pose.position
        self.goal_xy = (float(goal_pose.x), float(goal_pose.y))

    def cmd_move_cb(self, msg):
        new_cmd_move = bool(msg.data)
        if new_cmd_move and not self.active and self.goal_xy is not None and self.curr_xy is not None:
            self._begin_run()
        elif (not new_cmd_move) and self.active:
            status = "goal_reached" if self._distance_to_goal() <= self.goal_tolerance else "stopped"
            self._finish_run(status)
        self.cmd_move = new_cmd_move

    def local_plan_cb(self, msg):
        if not self.active or len(msg.data) < 2:
            return
        cmd_v = float(msg.data[0])
        cmd_omega = float(msg.data[1])
        self.cmd_v_sum += cmd_v
        self.cmd_v_count += 1
        self.max_abs_cmd_omega = max(self.max_abs_cmd_omega, abs(cmd_omega))

    def static_risk_cb(self, msg):
        if not self.active or self.curr_xy is None or len(msg.data) < 2:
            return

        best_dist = float("inf")
        for i in range(0, len(msg.data), 2):
            risk_x = float(msg.data[i])
            risk_y = float(msg.data[i + 1])
            if not math.isfinite(risk_x) or not math.isfinite(risk_y):
                continue
            dist = math.hypot(self.curr_xy[0] - risk_x, self.curr_xy[1] - risk_y)
            if dist < best_dist:
                best_dist = dist

        if math.isfinite(best_dist):
            self.min_static_risk_dist = min(self.min_static_risk_dist, best_dist)

    def on_shutdown(self):
        if self.active:
            self._finish_run("shutdown")


def main():
    rospy.init_node("experiment_logger")
    ExperimentLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
