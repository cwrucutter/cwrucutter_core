//steering
//
//Performs steering and path following for the CWRU Cutter
//robot. Will profile velocity and converge on the path.
//
//Parameters:
//  - ~v_max - maximum linear velocity
//  - ~w_max - maximum angular velocity
//  - ~v_a_max - maximum linear acceleration
//  - ~w_a_max - maximum angular acceleration
//Subscribes to:
//  - cwru/waypts (Path output from cutter_planner)
//  - cwru/slip (slipping monitor from... somewhere. Maybe Charles knows where?
//Publishes:
//  - cwru/cmd_vel

#include <ros/ros.h> //base libraries
#include <geometry_msgs/Twist.h> //data type for velocities
#include <tf/tf.h> // for tf::getYaw
#include "angles/angles.h" // part of tf stack, get angular distance etc
#include <cutter_arduino/Potentiometer.h>
#include <cutter_msgs/Switches.h>
#include <algorithm>

#define Hz 10
#define ROSCONSOLE_SEVERITY_DEBUG //messages at this level to be output


class CutterWhiskerSteering
{
  public:
    CutterWhiskerSteering();
    bool lookupParams();
    void steer();
  
  private:
    void switchCB(const cutter_msgs::Switches &controls);
    void whiskerCB(const cutter_arduino::Potentiometer &whiskers);
    
    double profile(double vw, double last_vw, double max_vw, double a);
    bool publishVW(double v, double w);

    ros::NodeHandle nh_;
  
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber switch_sub_;
    ros::Subscriber whisker_sub_;

    //parameters
    double v_max_;
    double w_max_;
    double v_a_max_;
    double w_a_max_;
    double PID_P_;
    double PID_D_;
    double PID_Set_;
    
    double whisker_right_;
    double whisker_left_;

    //member variables
    bool switchesgood_;
    geometry_msgs::Twist last_cmd_vel_;
};

void CutterWhiskerSteering::switchCB(const cutter_msgs::Switches &controls)
{
  //ROS_INFO("Got an updated switch status.");
  switchesgood_ = (controls.driveEnable && !controls.switchA);
}

void CutterWhiskerSteering::whiskerCB(const cutter_arduino::Potentiometer &whiskers)
{
  //ROS_INFO("Got an updated whisker reading.");
  whisker_right_ = whiskers.pot0;
  whisker_left_  = whiskers.pot1;
}

//##constructor##
CutterWhiskerSteering::CutterWhiskerSteering():
  v_max_(0.4), w_max_(0.4), v_a_max_(1.0), w_a_max_(1.0), PID_P_(1), PID_D_(0.1), PID_Set_(800), switchesgood_(true)//params(defaults)
{
  //set up subscribers
  whisker_sub_ = nh_.subscribe("cwru/whiskers",1,&CutterWhiskerSteering::whiskerCB,this);\
  switch_sub_ = nh_.subscribe("cwru/switches",1,&CutterWhiskerSteering::switchCB,this);

  //set up publisher
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
}


bool CutterWhiskerSteering::lookupParams()
{
  bool test = true;
  test = ros::param::get("~v_max", v_max_) && test;
  test = ros::param::get("~w_max", w_max_) && test;
  test = ros::param::get("~v_a_max", v_a_max_) && test;
  test = ros::param::get("~w_a_max", w_a_max_) && test;
  test = ros::param::get("~PID_P", PID_P_) && test;
  test = ros::param::get("~PID_D", PID_D_) && test;
  test = ros::param::get("~PID_Set", PID_Set_) && test;
  return test;
}


double CutterWhiskerSteering::profile(double vw, double last_vw, double max_vw, double a)
{
  if( vw > 0 )
    return 
      std::max(
        std::min(
          std::min(
            vw, 
            last_vw + a*(1.0/Hz)), 
          max_vw),
        last_vw-a*(1.0/Hz));
        
  else 
    return 
      std::min(
        std::max(
          std::max(
            vw,
            last_vw - a*(1.0/Hz)),
          -max_vw),
        last_vw+a*(1.0/Hz));
}

void CutterWhiskerSteering::steer()
{

  if(!switchesgood_){
    ROS_WARN("Steering is currently disabled. Make sure switch A is set to \"ITX cmd_vel\" and driving is enabled.");
    publishVW(0,0);
    return;
  }
  
  //local declarations
  double v, w;
  
  // Performs steering
  v = v_max_;
  w = PID_P_*(PID_Set_-whisker_right_)/1000; //Always turn right. Whisker_right_ varies from 1000 to 600. 

  //stay within limits
  w = profile(w, last_cmd_vel_.angular.z, w_max_, w_a_max_);
  v = profile(v, last_cmd_vel_.linear.x , v_max_, v_a_max_);

  ROS_INFO("profiled v: %f", v);

  publishVW(v,w);

  return;
}

bool CutterWhiskerSteering::publishVW(double v, double w)
{

  // create a message
  geometry_msgs::Twist steeringMsg;
  
  //set v, w. 
  steeringMsg.linear.x = v;
  steeringMsg.angular.z = w;

  //publish command velocity
  cmd_vel_pub_.publish(steeringMsg);

  //save everything important for next time
  //set last = target, taget = next, next = nextnext
  last_cmd_vel_ = steeringMsg;

  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "whisker_steering");
  
  CutterWhiskerSteering steering;

  ros::Rate loop_rate(Hz);

/*
  if (steering.lookupParams())
  {
    ROS_DEBUG("Parameters found");
  }
  else
  {
    ROS_WARN("Parameters not found");
  }*/

  while (ros::ok())
  {
    ros::spinOnce(); // trigger callbacks

    steering.steer(); // magic happens

    loop_rate.sleep(); // wait for next tick
  }

  return 0;
}
