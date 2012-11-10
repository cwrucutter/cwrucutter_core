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
#include <cutter_msgs/WayPoint.h>
#include <cutter_msgs/State.h>

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


    ros::NodeHandle nh_;
  
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber way_point_sub_;

	//parameters
    double v_max_;
    double w_max_;
    double v_a_max_;
    double w_a_max_;

	//member variables
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
  //target_waypoint_ = wp;

  ROS_DEBUG("Steering got updated waypoint information");
}


void CutterSteering::stateCB(const cutter_msgs::State &state)
{
  state_ = state;
  map_pose_ = state.pose.pose;

  ROS_DEBUG("Steering got updated state");
}


CutterSteering::CutterSteering():
  v_max_(1.0), w_max_(1.0), v_a_max_(1.0), w_a_max_(1.0) //params(defaults)
{

  //member variable intializations
  //no need to initialize params.
  cutter_msgs::WayPoint test_waypoint_;
	test_waypoint_.pose.position.x = 1.0;
	test_waypoint_.pose.position.y = 1.0;
	target_waypoint_ = test_waypoint_;


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
  return ros::param::get("~v_max", v_max_) && ros::param::get("~w_max", w_max_)
    && ros::param::get("~v_a_max", v_a_max_) && ros::param::get("~w_a_max", w_a_max_);
}


double CutterSteering::euclideanDistance(geometry_msgs::Point p, geometry_msgs::Point q)
{
  //3d euclidean distance
  return sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2));
}


void CutterSteering::steer()
{
  //local declarations
  double v, w;

  //check distance to WayPt.
  double targetDistance = euclideanDistance(map_pose_.position, last_waypoint_.pose.position);
  double xDistance = map_pose_.position.x - target_waypoint_.pose.position.x;
  double yDistance = map_pose_.position.y - target_waypoint_.pose.position.y;
  ROS_DEBUG("Robot (%.2f,%2f) distance to next waypoint (%.2f,%.2f): %f", map_pose_.position.x, map_pose_.position.y, target_waypoint_.pose.position.x, target_waypoint_.pose.position.y, targetDistance);

  double targetTheta = atan2(xDistance, yDistance);
  double robotTheta = tf::getYaw(map_pose_.orientation);
  ROS_DEBUG("Angle to target: %f \t\t Angle of robot: %f", targetTheta, robotTheta);

  //Are we there (x,y)?
  if (targetDistance < target_waypoint_.distanceTol)
  {
    ROS_DEBUG("We made it to the waypoint.");
    //find desired heading given Pose and NextWayPt
    //Are we at the desired heading?
    //if not, issue only w command (pivot)
    //set last = target, taget = next, next = nextnext
  }
  else
  {
    //Are we not there yet?
    //find desired heading given Pose and TargetWayPt
    //find difference in Pose-heading and desired-heading
    //find curvature. w = k*v?
  }

  //if line to left
  //control w

  //if line to right
  //*-1

  //stay within limits
  if( w > w_max_ )
    w = w_max_;
  else if ( w < -w_max_)
    w = -w_max_;

  // create a message
  geometry_msgs::Twist steeringMsg;
  
  //set v, w. 
  steeringMsg.linear.x = 0;
  steeringMsg.angular.z = 0;

  //publish command velocity
  //cmd_vel_pub_.publish(steeringMsg);
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
