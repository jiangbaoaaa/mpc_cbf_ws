#!/usr/bin/env python3

import math

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path


class GazeboPathVisualizer:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "global_path_marker")
        self.sample_stride = max(1, int(rospy.get_param("~sample_stride", 3)))
        self.segment_width = float(rospy.get_param("~segment_width", 0.08))
        self.segment_height = float(rospy.get_param("~segment_height", 0.03))
        self.point_radius = float(rospy.get_param("~point_radius", 0.06))
        self.z_offset = float(rospy.get_param("~z_offset", 0.03))
        self.segment_rgba = rospy.get_param("~segment_rgba", [0.1, 0.7, 1.0, 0.75])
        self.point_rgba = rospy.get_param("~point_rgba", [0.95, 0.35, 0.1, 0.95])

        self.last_signature = None
        self.path_spawned = False

        rospy.loginfo("Waiting for Gazebo model services for global path visualization...")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/delete_model")
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

        self.sub_path = rospy.Subscriber("/global_path", Path, self.path_callback, queue_size=1)

    def path_callback(self, msg):
        if not msg.poses:
            return

        sampled_points = []
        for idx, pose in enumerate(msg.poses):
            if idx % self.sample_stride == 0 or idx == len(msg.poses) - 1:
                sampled_points.append((pose.pose.position.x, pose.pose.position.y))

        if len(sampled_points) < 2:
            return

        signature = tuple((round(x, 3), round(y, 3)) for x, y in sampled_points)
        if signature == self.last_signature and self.path_spawned:
            return

        self.last_signature = signature
        model_xml = self.build_sdf(sampled_points)

        try:
            self.delete_model(self.model_name)
        except rospy.ServiceException:
            pass

        try:
            pose = Pose()
            pose.orientation.w = 1.0
            self.spawn_model(self.model_name, model_xml, "", pose, "world")
            self.path_spawned = True
            rospy.loginfo("Spawned Gazebo global path model with %d sampled points.", len(sampled_points))
        except rospy.ServiceException as exc:
            rospy.logwarn("Failed to spawn Gazebo global path model: %s", exc)

    def build_sdf(self, points):
        visuals = []

        for idx, (x, y) in enumerate(points):
            visuals.append(
                self.sphere_visual(
                    f"path_point_{idx}",
                    x,
                    y,
                    self.z_offset + self.segment_height * 0.6,
                    self.point_radius,
                    self.point_rgba,
                )
            )

        for idx in range(len(points) - 1):
            x0, y0 = points[idx]
            x1, y1 = points[idx + 1]
            dx = x1 - x0
            dy = y1 - y0
            length = math.hypot(dx, dy)
            if length < 1e-4:
                continue
            cx = 0.5 * (x0 + x1)
            cy = 0.5 * (y0 + y1)
            yaw = math.atan2(dy, dx)
            visuals.append(
                self.box_visual(
                    f"path_segment_{idx}",
                    cx,
                    cy,
                    self.z_offset,
                    yaw,
                    length,
                    self.segment_width,
                    self.segment_height,
                    self.segment_rgba,
                )
            )

        return (
            "<sdf version='1.6'>"
            "<model name='{name}'>"
            "<static>true</static>"
            "<link name='path_link'>"
            "{visuals}"
            "</link>"
            "</model>"
            "</sdf>"
        ).format(name=self.model_name, visuals="".join(visuals))

    def sphere_visual(self, name, x, y, z, radius, rgba):
        return (
            "<visual name='{name}'>"
            "<pose>{x} {y} {z} 0 0 0</pose>"
            "<geometry><sphere><radius>{radius}</radius></sphere></geometry>"
            "{material}"
            "</visual>"
        ).format(name=name, x=x, y=y, z=z, radius=radius, material=self.material_tag(rgba))

    def box_visual(self, name, x, y, z, yaw, length, width, height, rgba):
        return (
            "<visual name='{name}'>"
            "<pose>{x} {y} {z} 0 0 {yaw}</pose>"
            "<geometry><box><size>{length} {width} {height}</size></box></geometry>"
            "{material}"
            "</visual>"
        ).format(
            name=name,
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            length=length,
            width=width,
            height=height,
            material=self.material_tag(rgba),
        )

    def material_tag(self, rgba):
        r, g, b, a = rgba
        return (
            "<material>"
            "<ambient>{r} {g} {b} {a}</ambient>"
            "<diffuse>{r} {g} {b} {a}</diffuse>"
            "</material>"
        ).format(r=r, g=g, b=b, a=a)


if __name__ == "__main__":
    rospy.init_node("gazebo_path_visualizer")
    GazeboPathVisualizer()
    rospy.spin()
