#!/usr/bin/env python3

import math

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("reset_jackal_pose")

    model_name = rospy.get_param("~model_name", "jackal")
    x = rospy.get_param("~x", -5.5)
    y = rospy.get_param("~y", 0.0)
    z = rospy.get_param("~z", 0.15)
    yaw = rospy.get_param("~yaw", 0.0)
    cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
    publish_stop_count = int(rospy.get_param("~publish_stop_count", 8))

    stop_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1, latch=True)
    stop_cmd = Twist()
    for _ in range(publish_stop_count):
        stop_pub.publish(stop_cmd)
        rospy.sleep(0.02)

    rospy.wait_for_service("/gazebo/set_model_state")
    set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    state = ModelState()
    state.model_name = model_name
    state.reference_frame = "world"
    state.pose.position.x = x
    state.pose.position.y = y
    state.pose.position.z = z
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    state.pose.orientation.x = qx
    state.pose.orientation.y = qy
    state.pose.orientation.z = qz
    state.pose.orientation.w = qw
    state.twist = Twist()

    resp = set_model_state(state)
    if resp.success:
        rospy.loginfo("Reset %s to x=%.2f y=%.2f yaw=%.2f", model_name, x, y, yaw)
    else:
        rospy.logerr("Failed to reset %s: %s", model_name, resp.status_message)


if __name__ == "__main__":
    main()
