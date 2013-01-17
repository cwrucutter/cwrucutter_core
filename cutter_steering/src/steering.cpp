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
//#include <ros/roscpp.h> //base libraries
#include <nav_msgs/Path.h> // publish path for viz (TODO)
#include <geometry_msgs/Twist.h> //data type for velocities
#include <tf/tf.h> // for tf::getYaw
#include "angles/angles.h" // part of tf stack, get angular distance etc
#include <cutter_msgs/WayPoint.h>
#include <cutter_msgs/GetWayPoints.h>
#include <cutter_msgs/State.h>
#include <cutter_msgs/SlipStatus.h>
#include <cutter_msgs/Switches.h>
#include <algorithm>

#define Hz 10
#define ROSCONSOLE_SEVERITY_DEBUG //messages at this level to be output


class CutterSteering
{
  public:
    CutterSteering();
    bool lookupParams();
    bool getFirstWayPoint();
    void steer();
  
  private:
    void stateCB(const cutter_msgs::State &state);
    void wayPointCB(const cutter_msgs::WayPoint &wayPoint);
    void slipCB(const cutter_msgs::SlipStatus &slip);
    void switchCB(const cutter_msgs::Switches &controls);
  
    double euclideanDistance(geometry_msgs::Point p, geometry_msgs::Point q);
    double distToLine(double a1x, double a1y, geometry_msgs::Point, geometry_msgs::Point);
    double profile(double vw, double last_vw, double max_vw, double a);
    bool publishVW(double v, double w);

    ros::NodeHandle nh_;
  
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber switch_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber slip_sub_;
    ros::Subscriber way_point_sub_;

    //parameters
    double v_max_;
    double w_max_;
    double stall_time_;
    double v_a_max_;
    double w_a_max_;
    double K_wc_proportional_;
    double K_wth_proportional_;
    double K_v_proportional_;
    double w_deadband_;
    double v_deadband_;

    //member variables
    bool got_state_;
    bool got_waypoint_;
    bool initial_align;
    bool slipped_;
    bool sliprecover_;
    double pointofstall_;
    bool switchesgood_;
    geometry_msgs::Twist last_cmd_vel_;
    cutter_msgs::State last_state_;
    cutter_msgs::State state_;
    geometry_msgs::Pose last_map_pose_;
    geometry_msgs::Pose map_pose_;

    cutter_msgs::WayPoint test_waypoint_;
    cutter_msgs::WayPoint last_waypoint_;
    cutter_msgs::WayPoint target_waypoint_;
    cutter_msgs::WayPoint next_waypoint_;

    ros::ServiceClient waypoint_client_;
    cutter_msgs::GetWayPoints waypoint_srv_;

    double initialX;
    double initialY;

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

void CutterSteering::slipCB(const cutter_msgs::SlipStatus &slipstat)
{
  ROS_INFO("Got an updated slip status of %i", slipstat.slip);
  slipped_ = (slipstat.slip == 1);
}

void CutterSteering::switchCB(const cutter_msgs::Switches &controls)
{
  ROS_INFO("Got an updated switch status.");
  switchesgood_ = (controls.driveEnable && !controls.switchA);
}

//##constructor##
CutterSteering::CutterSteering():
  v_max_(1.0), w_max_(1.0), v_a_max_(1.0), w_a_max_(1.0), K_wc_proportional_(0.1),K_wth_proportional_(0.1), v_deadband_(.001), w_deadband_(.001), switchesgood_(true)//params(defaults)
{

  //member variable initializations
  //no need to initialize params.
  cutter_msgs::WayPoint test_waypoint_;
  test_waypoint_.pose.position.x = 1.0;
  test_waypoint_.pose.position.y = 1.0;
  //target_waypoint_ = test_waypoint_;
  map_pose_.orientation = tf::createQuaternionMsgFromYaw(0.0);
  got_state_ = false;
  got_waypoint_ = false;

  //set up subscribers
  state_sub_ = nh_.subscribe("cwru/state",1,&CutterSteering::stateCB,this);
  way_point_sub_ = nh_.subscribe("cwru/waypoint",1,&CutterSteering::wayPointCB,this);
  slip_sub_ = nh_.subscribe("cwru/slip",1,&CutterSteering::slipCB,this);
  switch_sub_ = nh_.subscribe("cwru/switches",1,&CutterSteering::switchCB,this);

  //set up publisher
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);

  //set up client for waypoints from path planner
  waypoint_client_ = nh_.serviceClient<cutter_msgs::GetWayPoints>("get_waypoint");

  //reord initial position for line creation
  //initial_X = map_pose.pose.position.x;

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


bool CutterSteering::getFirstWayPoint()
{
  if (got_waypoint_)
    return true;

  waypoint_srv_.request.increment = false;
  if(waypoint_client_.call(waypoint_srv_))
  {
    if (waypoint_srv_.response.pointsLeft > 0)
    {
      got_waypoint_ = true; 
      ROS_INFO("Got first waypoint");

      initialX = map_pose_.position.x;
      initialY = map_pose_.position.y;
      ROS_INFO("Recorded initial position.");

      last_waypoint_ = cutter_msgs::WayPoint();
      target_waypoint_ = waypoint_srv_.response.currWayPoint;
      next_waypoint_ = waypoint_srv_.response.nextWayPoint;
      initial_align = true;

      return true;
    }
  }
  return false;
}


bool CutterSteering::lookupParams()
{
  bool test = true;
  test = ros::param::get("~v_max", v_max_) && test;
  test = ros::param::get("~stall_time", stall_time_) && test;
  test = ros::param::get("~w_max", w_max_) && test;
  test = ros::param::get("~v_a_max", v_a_max_) && test;
  test = ros::param::get("~w_a_max", w_a_max_) && test;
  test = ros::param::get("~K_wc_proportional", K_wc_proportional_) && test;
  test = ros::param::get("~K_wth_proportional", K_wth_proportional_) && test;
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

double CutterSteering::distToLine(double a1x, double a1y, geometry_msgs::Point target, geometry_msgs::Point robot){//This function returns the distance to the line between the robot's initial position and the waypoint position. TODO: Replace the duplicate math with however c++ handles mathematical vectors.
  double avecx = target.x- a1x;
  double avecy = target.y - a1y;
  double bvecx = robot.x- a1x;
  double bvecy = robot.y - a1y;
  double anorm = sqrt(pow(avecx, 2) + pow(avecy, 2) );//Square-root distance formula.
  if(anorm == 0.0) return 0.0;
  double lnorm = (avecx*bvecx + avecy*bvecy)/anorm;//Dot-product divided by norm
  double ahatx = avecx/anorm;
  double ahaty = avecy/anorm;
  double lx = lnorm * ahatx;
  double ly = lnorm * ahaty;
  double cx = bvecx - lx;
  double cy = bvecy - ly;
  double cnorm = sqrt(pow(cx, 2) + pow(cy, 2) );
  double theta = atan2(avecy, avecx);
  double thetb = atan2(bvecy, bvecx);
  double thet1 = angles::shortest_angular_distance(theta, thetb);
  if(thet1 == 0.0) return 0.0;
  double d = -cnorm * (thet1 / fabs(thet1));
  return d;
}

void CutterSteering::steer()
{

  if(!switchesgood_){
    ROS_WARN("Steering is currently disabled. Make sure switch A is set to \"ITX cmd_vel\" and driving is enabled.");
    publishVW(0,0);
    return;
  }
  if (!got_state_)
  {
    ROS_WARN("Steering tried to steer, but hasn't got a state");
    publishVW(0,0);
    return;
  }

  if (!getFirstWayPoint())
  {
    ROS_WARN("Steering tried to steer, but has no more waypoints");
    publishVW(0,0);
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

  ROS_INFO("Robot (%.2f,%2f), Intitial position of robot was (%.2f,%.2f), distance to next waypoint (%.2f,%.2f): %f", map_pose_.position.x, map_pose_.position.y, initialX, initialY, target_waypoint_.pose.position.x, target_waypoint_.pose.position.y, targetDistance);

  /*if (false)//TODO: Tom has no idea what this thing is. The velocity will be set to 5 if it is triggered, causing the robot to spiral out of control. (Originally the if statement contained "yDistance == 0 || xDistance ==0"
  {
    ROS_WARN("Starting up? xDistance or yDistance from robot to waypoint is 0!!");
    targetTheta = 0;
  }
  else
  {*/
    targetTheta = atan2(yDistance * (target_waypoint_.direction - 2.0), xDistance * (target_waypoint_.direction - 2.0));
  //}

  robotTheta = tf::getYaw(map_pose_.orientation);
  ROS_INFO("Angle to target: %f \t\t Angle of robot: %f", targetTheta, robotTheta);


  //time check: older than .5s? then don't go so fast!
  if(state_.header.stamp - ros::Time::now() > ros::Duration(.5)) //half a second
  {
    ROS_WARN("State is out of date!!");
    w = 0;
    v = 0;
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

      waypoint_srv_.request.increment = true;
      if(waypoint_client_.call(waypoint_srv_))
      {
        if (waypoint_srv_.response.pointsLeft > 0)
        {
          got_waypoint_ = true; 
          ROS_INFO("Got next waypoint");

          last_waypoint_ = target_waypoint_;
          target_waypoint_ = waypoint_srv_.response.currWayPoint;
          next_waypoint_ = waypoint_srv_.response.nextWayPoint;
	  ROS_INFO("Updating initials");
	  initialX = last_waypoint_.pose.position.x;
          initialY = last_waypoint_.pose.position.y;
	  initial_align = true;
        }
        else
        {
          ROS_INFO("No more waypoints");
          got_waypoint_ = false; 
        }
      }
      else
      {
        ROS_WARN("Couldn't contact Path Planner's waypoint service");
        got_waypoint_ = false; 
      }
    }
    else
    {
      //Are we not there yet?
      //find desired heading given Pose and TargetWayPt
      //find difference in Pose-heading and desired-heading

      thetaDesired = targetTheta; // for now just want to aim at target (good for intial align
      thetaError = angles::shortest_angular_distance(thetaDesired, robotTheta); // ros tf angles
      ROS_INFO("found theta diff: %f", thetaError);
      if(initial_align){//If the robot has not begun its jorney to the target, we want it to turn and face said target.
	ROS_INFO("In initial rotational alignment stage.");
	v = 0.0;
        w = -K_wth_proportional_*thetaError;
        ROS_INFO("Set w: %f", w);
	initial_align = (fabs(thetaError) > 0.1);
      }
      else
      {
	    double c = CutterSteering::distToLine(initialX, initialY, target_waypoint_.pose.position, map_pose_.position);
        ROS_INFO("c = %f", c);
        v = targetDistance*K_v_proportional_;
	    w = K_wc_proportional_ * c - K_wth_proportional_ * thetaError;
	    ROS_INFO("Progressing to target.");
	if(slipped_){
	    ROS_WARN("Stalled!!!!!");
            pointofstall_ = ros::Time::now().toSec();
            ROS_INFO("Time of stall was %f.", pointofstall_);
	    slipped_ = false;
	    sliprecover_ = true;
	}
	if(sliprecover_){
	    w = 0.0;
	    v = -1.0 * v;
	    if(ros::Time::now().toSec() > pointofstall_ + stall_time_){
		sliprecover_ = false;
	    }
	}
      }

    }
    
    //stay within limits
    w = profile(w, last_cmd_vel_.angular.z, w_max_, w_a_max_);
    v = profile(v * (target_waypoint_.direction - 2.0), last_cmd_vel_.linear.x, v_max_, v_a_max_);

    ROS_INFO("profiled v: %f", v);
  }

  publishVW(v,w);

  return;
}

bool CutterSteering::publishVW(double v, double w)
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
