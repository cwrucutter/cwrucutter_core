//steering
//
//Performs steering and path following for the CWRU Cutter
//robot. Will profile velocity and converge on the path.
//
//Parameters:
//  - 
//Subscribes to:
//  - cwru/waypts (Path output from cutter_planner)
//
//Publishes:
//  - cwru/cmd_vel

#include <ros/ros.h> //base libraries
#include <nav_msgs/Odometry.h> // ???
#include <nav_msgs/Path.h> // publish path for viz 
#include <geometry_msgs/PoseStamped.h> // ???
#include <geometry_msgs/Twist.h> //data type for velocities
#include <tf/transform_datatypes.h> // for tf::getYaw
#include <cutter_msgs/WayPoint.h>
#include <cutter_msgs/State.h>

#define Hz 10

class CutterSteering
{
  public:
    CutterSteering();
  
  private:
    void stateCB(const cutter_msgs::State::constPtr& state);
    void wayPointCB(const cutter_msgs::WayPoint::constPtr& wayPoint);
  
    ros::nodehandle nh_;
  
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber way_points_sub_;

    nav_msgs::Odometry last_odom_;
    geometry_msgs::PoseStamped last_map_pose_;

};

CutterSteering::CutterSteering():
 //params
{

  //member variable intializations
  //last timestep
  //current timestep

  
  //set up subscribers
  state_sub_ = nh_.subscribe<cutter_msgs::State>("cwru/state",1,stateCB);
  way_points_sub_ = nh_.subscribe<cutter_msgs::WayPoints>("cwru/waypoints",1,wayPointCB);

  //set up publisher
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);

  //call steering algorithm?
}


bool CutterOdometry::lookupParams()
{
  return true;
      //&& ros::param::get("~tpm_right",ticks_per_m_right_);
}

double euclideanDistance(geometry_msgs::Point p, geometry_msgs::Point q)
{
  //3d euclidean distance
  return math.sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2));
}


void CutterSteering::steerToPoint(
                                  //Pose
                                  geometry_msgs::PoseStamped map_pose,
                                  //LastWayPt
                                  cutter_msgs::WayPoint last_waypoint,
                                  //TargetWayPt
                                  cutter_msgs::WayPoint target_waypoint,
                                  //NextWayPt
                                  cutter_msgs::WayPoint next_waypoint,
                                  //vMax
                                  //wMax
                                  //vaMax
                                  //waMax
                                  )
{

  geometry_msgs::Twist steeringMsg;

  //check distance to WayPt.
  double targetDistance = euclidianDistance(map_pose.pose.position, last_waypoint.pose.position);
  double xDistance = map_pose.pose.position.x - target_waypoint.pose.position.x;
  double yDistance = map_pose.pose.position.y - target_waypoint.pose.position.y;

  double targetTheta = math.atan2(xDistance, yDistance);
  double robotTheta = tf::getYaw(map_pose.pose.orientation);
  

  //Are we there (x,y)?
  if (targetDistance < target_waypoint.distanceTol)
  {
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
  //control on w

  //if line to right
  //*-1

  //set v, w. 
  steeringMsg.linear.x = 0;
  steeringMsg.angular.z = 0;

  //publish command velocity
  cmd_vel_pub_.publish(steeringMsg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "steering");
  
  CutterSteering steering;

  ros::Rate loop_rate(Hz);

  while (ros::ok())
  {
    loop_rate.sleep();
  }

  return 0;
}
~
