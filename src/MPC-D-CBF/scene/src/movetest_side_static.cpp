#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include "moving_cylinder.hpp"

namespace
{
constexpr uint8_t kCylinderNum = 1;

bool g_is_move = false;
bool g_have_curr_state = false;
double g_robot_x = 0.0;
double g_trigger_x = 4.5;

void cmdCallback(const std_msgs::Bool::ConstPtr& cmd_move)
{
  g_is_move = cmd_move->data;
}

void currStateCallback(const std_msgs::Float32MultiArray::ConstPtr& curr_state)
{
  if (curr_state->data.empty())
    return;

  g_robot_x = curr_state->data[0];
  g_have_curr_state = true;
}
}  // namespace

int MovingCylinder::id_ = 0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_side_static_model");
  ros::NodeHandle nh("~");
  nh.param<double>("trigger_x", g_trigger_x, 4.5);

  ros::Subscriber sub = nh.subscribe<std_msgs::Bool>("/cmd_move", 10, cmdCallback);
  ros::Subscriber curr_state_sub = nh.subscribe<std_msgs::Float32MultiArray>("/curr_state", 10, currStateCallback);
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState set_model_state_srv;

  MovingCylinder cylinder[kCylinderNum];
  geometry_msgs::Point init_pos[kCylinderNum];
  geometry_msgs::Twist twist[kCylinderNum];

  for (uint8_t i = 0; i < kCylinderNum; i++)
  {
    nh.param<double>("x" + std::to_string(i), init_pos[i].x, 6.5);
    nh.param<double>("y" + std::to_string(i), init_pos[i].y, -2.0);
    nh.param<double>("z" + std::to_string(i), init_pos[i].z, 0.25);
    nh.param<double>("vx" + std::to_string(i), twist[i].linear.x, 0.0);
    nh.param<double>("vy" + std::to_string(i), twist[i].linear.y, 0.006);

    cylinder[i].setPosition(init_pos[i]);
    cylinder[i].setVel(twist[i]);
  }

  while (ros::ok())
  {
    if (g_is_move && g_have_curr_state && g_robot_x >= g_trigger_x)
    {
      for (uint8_t i = 0; i < kCylinderNum; i++)
        cylinder[i].updateState();
    }

    for (uint8_t i = 0; i < kCylinderNum; i++)
    {
      set_model_state_srv.request.model_state = cylinder[i].model_state_;
      client.call(set_model_state_srv);
    }

    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  return 0;
}
