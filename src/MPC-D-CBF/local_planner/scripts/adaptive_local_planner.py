#!/usr/bin/env python3
"""Adaptive local planner: thin wrapper that delegates to MyLocalPlanner."""

import rospy
from my_local_planner import MyLocalPlanner

if __name__ == "__main__":
    rospy.init_node("adaptive_local_planner")
    planner = MyLocalPlanner()
    rospy.spin()
