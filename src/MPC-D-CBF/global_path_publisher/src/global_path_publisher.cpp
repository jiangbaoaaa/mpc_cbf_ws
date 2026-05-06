#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <sstream>
#include <string>
#include <vector>

using namespace Eigen;

ros::Publisher global_path_pub;
ros::Publisher global_goal_pub;
ros::Publisher global_path_vis_pub;

namespace
{
Vector2d start_pos;
Vector2d target_pos;

std::vector<Vector2d> parseWaypoints(const std::string& raw_waypoints)
{
  std::vector<Vector2d> waypoints;
  std::stringstream waypoint_stream(raw_waypoints);
  std::string token;

  while (std::getline(waypoint_stream, token, ';'))
  {
    if (token.empty())
    {
      continue;
    }

    std::stringstream point_stream(token);
    std::string x_token;
    std::string y_token;
    if (!std::getline(point_stream, x_token, ',') || !std::getline(point_stream, y_token, ','))
    {
      ROS_WARN_STREAM("Skip malformed waypoint token: " << token);
      continue;
    }

    try
    {
      waypoints.emplace_back(std::stod(x_token), std::stod(y_token));
    }
    catch (const std::exception& e)
    {
      ROS_WARN_STREAM("Skip malformed waypoint token: " << token << " (" << e.what() << ")");
    }
  }

  return waypoints;
}

std::vector<Vector2d> buildControlPoints(ros::NodeHandle& node)
{
  node.param<double>("start_x", start_pos.x(), -10.0);
  node.param<double>("start_y", start_pos.y(), 0.0);
  node.param<double>("end_x", target_pos.x(), 9.0);
  node.param<double>("end_y", target_pos.y(), 0.0);

  std::string raw_waypoints;
  node.param<std::string>("path_waypoints", raw_waypoints, "");
  std::vector<Vector2d> waypoints = parseWaypoints(raw_waypoints);
  if (waypoints.size() >= 2)
  {
    start_pos = waypoints.front();
    target_pos = waypoints.back();
    return waypoints;
  }

  if (!raw_waypoints.empty())
  {
    ROS_WARN("Falling back to straight global path because path_waypoints could not be parsed.");
  }

  return {start_pos, target_pos};
}

nav_msgs::Path buildPath(const std::vector<Vector2d>& control_points, double step)
{
  nav_msgs::Path global_path;
  global_path.header.stamp = ros::Time::now();
  global_path.header.frame_id = "world";

  if (control_points.empty())
  {
    return global_path;
  }

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = global_path.header.stamp;
  pose_stamped.header.frame_id = global_path.header.frame_id;
  pose_stamped.pose.orientation.w = 1.0;

  int seq = 0;
  auto push_waypoint = [&](const Vector2d& waypoint) {
    pose_stamped.header.seq = seq++;
    pose_stamped.pose.position.x = waypoint.x();
    pose_stamped.pose.position.y = waypoint.y();
    pose_stamped.pose.position.z = 0.0;
    global_path.poses.push_back(pose_stamped);
  };

  push_waypoint(control_points.front());

  for (size_t idx = 1; idx < control_points.size(); ++idx)
  {
    const Vector2d segment = control_points[idx] - control_points[idx - 1];
    const double dist = segment.norm();
    if (dist < 1e-6)
    {
      continue;
    }

    const Vector2d direction = segment / dist;
    for (double travelled = step; travelled < dist; travelled += step)
    {
      push_waypoint(control_points[idx - 1] + travelled * direction);
    }
    push_waypoint(control_points[idx]);
  }

  return global_path;
}

Vector2d catmullRomPoint(const Vector2d& p0,
                         const Vector2d& p1,
                         const Vector2d& p2,
                         const Vector2d& p3,
                         double t)
{
  const double t2 = t * t;
  const double t3 = t2 * t;
  return 0.5 * ((2.0 * p1) + (-p0 + p2) * t + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * t2 +
                (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * t3);
}

nav_msgs::Path buildSmoothPath(const std::vector<Vector2d>& control_points, double step)
{
  nav_msgs::Path global_path;
  global_path.header.stamp = ros::Time::now();
  global_path.header.frame_id = "world";

  if (control_points.empty())
  {
    return global_path;
  }

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = global_path.header.stamp;
  pose_stamped.header.frame_id = global_path.header.frame_id;
  pose_stamped.pose.orientation.w = 1.0;

  int seq = 0;
  auto push_waypoint = [&](const Vector2d& waypoint) {
    pose_stamped.header.seq = seq++;
    pose_stamped.pose.position.x = waypoint.x();
    pose_stamped.pose.position.y = waypoint.y();
    pose_stamped.pose.position.z = 0.0;
    global_path.poses.push_back(pose_stamped);
  };

  push_waypoint(control_points.front());

  for (size_t idx = 0; idx + 1 < control_points.size(); ++idx)
  {
    const Vector2d& p0 = control_points[idx == 0 ? idx : idx - 1];
    const Vector2d& p1 = control_points[idx];
    const Vector2d& p2 = control_points[idx + 1];
    const Vector2d& p3 = control_points[idx + 2 < control_points.size() ? idx + 2 : idx + 1];

    const double dist = (p2 - p1).norm();
    const int samples = std::max(2, static_cast<int>(std::ceil(dist / std::max(step, 1e-3))));
    for (int sample = 1; sample <= samples; ++sample)
    {
      const double t = static_cast<double>(sample) / static_cast<double>(samples);
      push_waypoint(catmullRomPoint(p0, p1, p2, p3, t));
    }
  }

  return global_path;
}

visualization_msgs::Marker buildGoalMarker()
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";
  marker.ns = "global_goal";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = target_pos.x();
  marker.pose.position.y = target_pos.y();
  marker.pose.position.z = 0.35;
  marker.scale.x = 0.35;
  marker.scale.y = 0.35;
  marker.scale.z = 0.35;
  marker.color.r = 1.0;
  marker.color.g = 0.85;
  marker.color.b = 0.1;
  marker.color.a = 0.95;
  marker.lifetime = ros::Duration(0.0);
  return marker;
}

visualization_msgs::MarkerArray buildPathMarkers(const std::vector<Vector2d>& control_points,
                                                 const nav_msgs::Path& global_path)
{
  visualization_msgs::MarkerArray marker_array;

  visualization_msgs::Marker line_marker;
  line_marker.header.stamp = ros::Time::now();
  line_marker.header.frame_id = "world";
  line_marker.ns = "global_path";
  line_marker.id = 0;
  line_marker.type = visualization_msgs::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = 0.16;
  line_marker.color.r = 0.10;
  line_marker.color.g = 0.70;
  line_marker.color.b = 1.00;
  line_marker.color.a = 0.95;
  line_marker.lifetime = ros::Duration(0.0);

  for (const auto& pose : global_path.poses)
  {
    geometry_msgs::Point point;
    point.x = pose.pose.position.x;
    point.y = pose.pose.position.y;
    point.z = 0.08;
    line_marker.points.push_back(point);
  }
  marker_array.markers.push_back(line_marker);

  visualization_msgs::Marker waypoint_marker;
  waypoint_marker.header = line_marker.header;
  waypoint_marker.ns = "global_waypoints";
  waypoint_marker.id = 1;
  waypoint_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  waypoint_marker.action = visualization_msgs::Marker::ADD;
  waypoint_marker.pose.orientation.w = 1.0;
  waypoint_marker.scale.x = 0.24;
  waypoint_marker.scale.y = 0.24;
  waypoint_marker.scale.z = 0.24;
  waypoint_marker.color.r = 0.95;
  waypoint_marker.color.g = 0.35;
  waypoint_marker.color.b = 0.10;
  waypoint_marker.color.a = 0.95;
  waypoint_marker.lifetime = ros::Duration(0.0);

  for (const auto& waypoint : control_points)
  {
    geometry_msgs::Point point;
    point.x = waypoint.x();
    point.y = waypoint.y();
    point.z = 0.18;
    waypoint_marker.points.push_back(point);
  }
  marker_array.markers.push_back(waypoint_marker);

  return marker_array;
}
}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "global_path_publisher");
  ros::NodeHandle node("~");

  global_path_pub = node.advertise<nav_msgs::Path>("/global_path", 1, true);
  global_goal_pub = node.advertise<visualization_msgs::Marker>("/global_goal_marker", 1, true);
  global_path_vis_pub = node.advertise<visualization_msgs::MarkerArray>("/global_path_vis", 1, true);

  double step = 0.1;
  node.param<double>("path_step", step, 0.1);
  bool smooth_path = false;
  node.param<bool>("smooth_path", smooth_path, false);

  const std::vector<Vector2d> control_points = buildControlPoints(node);
  const nav_msgs::Path global_path =
      (smooth_path && control_points.size() >= 3) ? buildSmoothPath(control_points, step)
                                                  : buildPath(control_points, step);
  const visualization_msgs::Marker goal_marker = buildGoalMarker();
  const visualization_msgs::MarkerArray path_markers = buildPathMarkers(control_points, global_path);

  global_path_pub.publish(global_path);
  global_goal_pub.publish(goal_marker);
  global_path_vis_pub.publish(path_markers);

  ros::spin();

  return 0;
}
