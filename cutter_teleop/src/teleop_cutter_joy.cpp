//teleop_cutter_joy
//
//based on http://www.ros.org/wiki/joy/Tutorials/WritingTeleopNode
//


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
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
  double linear_scale_;     // max linear velocity (m/s)
  double angular_scale_;    // max angular velocity (rad/s)

  ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;
};

//
TeleopCutter::TeleopCutter():
  linear_axis_(1), angular_axis_(3), linear_scale_(1), angular_scale_(1)//Joystick axes and maximum values for scaling
{
  ros::NodeHandle nhPrivate("~");
  nhPrivate.param("linear_axis",  linear_axis_, linear_axis_);
  nhPrivate.param("angular_axis", angular_axis_, angular_axis_);
  nhPrivate.param("linear_scale", linear_scale_, linear_scale_);
  nhPrivate.param("angular_scale", angular_scale_, angular_scale_);

  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("throttled_joy", 10, &TeleopCutter::joyCallback, this); //10 Hz
}

void TeleopCutter::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  double linear_cmd    = linear_scale_  * joy->axes[linear_axis_];
  double angular_cmd   = angular_scale_ * joy->axes[angular_axis_];

  ROS_INFO("Linear: %d, Angular: %d", linear_cmd, angular_cmd);

  vel.linear.x  = linear_cmd;
  vel.angular.z = angular_cmd;
  cmd_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_cutter");
  TeleopCutter teleoperatedLawnmower;

  ros::spin();
}
