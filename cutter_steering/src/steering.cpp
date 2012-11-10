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
//
//Publishes:
//  - cwru/cmd_vel

#include <ros/ros.h> //base libraries
#include <nav_msgs/Path.h> // publish path for viz (TODO)
#include <geometry_msgs/Twist.h> //data type for velocities
#include <tf/tf.h> // for tf::getYaw
#include "angles/angles.h" // part of tf stack, get angular distance etc
#include <cutter_msgs/WayPoint.h>
#include <cutter_msgs/State.h>
#include <algorithm>

#define Hz 10
#define ROSCONSOLE_SEVERITY_DEBUG //messages at this level to be output


class CutterSteering
{
  public:
    CutterSteering();
    bool lookupParams();
    void steer();
  
  private:
    void stateCB(const cutter_msgs::State &state);
    void wayPointCB(const cutter_msgs::WayPoint &wayPoint);
  
    double euclideanDistance(geometry_msgs::Point p, geometry_msgs::Point q);
    double profile(double vw, double last_vw, double max_vw, double a);

    ros::NodeHandle nh_;
  
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber way_point_sub_;

    //parameters
    double v_max_;
    double w_max_;
    double v_a_max_;
    double w_a_max_;
    double K_w_proportional_;
    double K_v_proportional_;
    double w_deadband_;
    double v_deadband_;

    //member variables
    bool got_state_;
    geometry_msgs::Twist last_cmd_vel_;
    cutter_msgs::State last_state_;
    cutter_msgs::State state_;
    geometry_msgs::Pose last_map_pose_;
    geometry_msgs::Pose map_pose_;

    cutter_msgs::WayPoint test_waypoint_;
    cutter_msgs::WayPoint last_waypoint_;
    cutter_msgs::WayPoint target_waypoint_;
    cutter_msgs::WayPoint next_waypoint_;

};


void CutterSteering::wayPointCB(const cutter_msgs::WayPoint &wp)
{
  target_waypoint_ = wp;

  ROS_INFO("Steering got updated waypoint information");
}


void CutterSteering::stateCB(const cutter_msgs::State &state)
{
  state_ = state;
  map_pose_ = state.pose.pose;

  ROS_INFO("Steering got updated state");
  got_state_ = true;
}

//##constructor##
CutterSteering::CutterSteering():
  v_max_(1.0), w_max_(1.0), v_a_max_(1.0), w_a_max_(1.0), K_w_proportional_(0.1), v_deadband_(.001), w_deadband_(.001) //params(defaults)
{

  //member variable intializations
  //no need to initialize params.
  cutter_msgs::WayPoint test_waypoint_;
  test_waypoint_.pose.position.x = 1.0;
  test_waypoint_.pose.position.y = 1.0;
  //target_waypoint_ = test_waypoint_;
  map_pose_.orientation = tf::createQuaternionMsgFromYaw(0.0);
  got_state_ = false;

  //last timestep

  //current timestep

  //set up subscribers
  state_sub_ = nh_.subscribe("cwru/state",1,&CutterSteering::stateCB,this);
  way_point_sub_ = nh_.subscribe("cwru/waypoint",1,&CutterSteering::wayPointCB,this);

  //set up publisher
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);

  //wait for transformers before letting anything else happen in this node
  /*
  tfl = new tf::TransformListener();
  while (!tfl->canTransform("map", "base_link", ros::Time::now()))
  { 
    ROS_INFO("Steering is WAITING for transformers... "); 
    ros::spinOnce();
  } 
  */
}


bool CutterSteering::lookupParams()
{
  bool test = true;
  test = ros::param::get("~v_max", v_max_) && test;
  test = ros::param::get("~w_max", w_max_) && test;
  test = ros::param::get("~v_a_max", v_a_max_) && test;
  test = ros::param::get("~w_a_max", w_a_max_) && test;
  test = ros::param::get("~K_w_proportional", K_w_proportional_) && test;
  test = ros::param::get("~K_v_proportional", K_v_proportional_) && test;
  //test = ros::param::get("~w_deadband", w_deadband_) && test;
  //test = ros::param::get("~v_deadband", v_deadband_) && test;
  return test;
}

double CutterSteering::profile(double vw, double last_vw, double max_vw, double a)
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

double CutterSteering::euclideanDistance(geometry_msgs::Point p, geometry_msgs::Point q)
{
  //3d euclidean distance
  return sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2));
}


void CutterSteering::steer()
{
  if (!got_state_)
  {
    ROS_WARN("Steering tried to steer, but hasn't got a state");
    return;
  }

  //local declarations
  double v, w;
  double robotTheta, targetTheta, thetaDesired, thetaError;

  //check distance to WayPt.
  double targetDistance = euclideanDistance(map_pose_.position, target_waypoint_.pose.position);
  double xDistance = target_waypoint_.pose.position.x - map_pose_.position.x;
  double yDistance = target_waypoint_.pose.position.y - map_pose_.position.y;

//  double brakingDistance = (last_cmd_vel_.linear.x/a_max_)*(last_v_/2.0); // (m/s)/(m/s/s)*m/s/2) = m/2 

  ROS_INFO("Robot (%.2f,%2f) distance to next waypoint (%.2f,%.2f): %f", map_pose_.position.x, map_pose_.position.y, target_waypoint_.pose.position.x, target_waypoint_.pose.position.y, targetDistance);

  if (xDistance == 0 || yDistance == 0)
  {
    ROS_WARN("Starting up? xDistance or yDistance from robot to waypoint is 0!!");
    targetTheta = 0;
  }
  else
  {
    targetTheta = atan2(yDistance, xDistance);
  }

  robotTheta = tf::getYaw(map_pose_.orientation);
  ROS_INFO("Angle to target: %f \t\t Angle of robot: %f", targetTheta, robotTheta);


  //time check: older than .5s? then don't go so fast!
  if(state_.header.stamp - ros::Time::now() > ros::Duration(.5)) //half a second
  {
    ROS_WARN("State is out of date!!");
    w = 0;
  }
  else
  {
    //control:
    //Are we there (x,y)?
    if (targetDistance < target_waypoint_.distanceTol)
    {
      ROS_INFO("We made it to the waypoint.");

      //don't move until a new waypoint is issued.
      v = 0;
      w = 0;

      //call for new waypoint from path planner
    }
    else
    {
      //Are we not there yet?
      //find desired heading given Pose and TargetWayPt
      thetaDesired = targetTheta; // for now just want to aim at target (TODO: remove stopgap measures)
      //find difference in Pose-heading and desired-heading
      thetaError = angles::shortest_angular_distance(thetaDesired, robotTheta); // ros tf angles
      ROS_INFO("found theta diff: %f", thetaError);

      if (fabs(thetaError) > 0.1)
      {
        v = 0;
      }
      else
      {
        //if (targetDistance < v_a_max_) //if the robot is close enough to 
        v = targetDistance*K_v_proportional_;
      }

      w = -K_w_proportional_*thetaError;
      ROS_INFO("Set w: %f", w);
    }
    
    //stay within limits
    w = profile(w, last_cmd_vel_.angular.z, w_max_, w_a_max_);
    v = profile(v, last_cmd_vel_.linear.x, v_max_, v_a_max_);

    ROS_INFO("profiled w: %f", w);
  }

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
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "steering");
  
  CutterSteering steering;

  ros::Rate loop_rate(Hz);

  if (steering.lookupParams())
  {
    ROS_DEBUG("Parameters found");
  }
  else
  {
    ROS_WARN("Parameters not found");
  }

  while (ros::ok())
  {
    ros::spinOnce(); // trigger callbacks

    steering.steer(); // magic happens

    loop_rate.sleep(); // wait for next tick
  }

  return 0;
}
