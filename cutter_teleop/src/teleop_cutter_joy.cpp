//based on http://www.ros.org/wiki/joy/Tutorials/WritingTeleopNode

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <cutter_msgs/VelCmd.h>
#include <ros/console.h>
#include <string>

class TeleopCutter
{
public:
  TeleopCutter();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
  ros::NodeHandle nh_;

  int linear_axis_, angular_axis_;
  double linear_max_;     // max linear velocity (m/s)
  double angular_max_;    // max angular velocity (rad/s)

  ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;
};


TeleopCutter::TeleopCutter():
  linear_axis_(1), angular_axis_(3), linear_max_(1), angular_max_(1)
{
  nh_.param("linear_axis",  linear_axis_, linear_axis_);
  nh_.param("angular_axis", angular_axis_, angular_axis_);
  nh_.param("linear_speed_max", linear_max_, linear_max_);
  nh_.param("angular_speed_max", angular_max_, angular_max_);

  cmd_pub_ = nh_.advertise<cutter_msgs::VelCmd>("cwru/cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("throttled_joy", 10, &TeleopCutter::joyCallback, this);
}

void TeleopCutter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  cutter_msgs::VelCmd vel;
  int linear_cmd    = 1000 * linear_max_  * joy->axes[linear_axis_];
  int angular_cmd   = 1000 * angular_max_ * joy->axes[angular_axis_];

  ROS_INFO("Linear: %d, Angular: %d", linear_cmd, angular_cmd);

  vel.linear  = linear_cmd;
  vel.angular = angular_cmd;
  cmd_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_cutter");
  TeleopCutter teleoperatedLawnmower;

  ros::spin();
}
