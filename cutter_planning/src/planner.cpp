// CWRU cutter path planner

#include <ros/ros.h>
#include "cutter_msgs/GetWayPoints.h"
#include <tf/tf.h>
#include <string>


class CutterPlanner
{
  public:
    CutterPlanner();

  private:
    bool getWayPointService(cutter_msgs::GetWayPoints::Request &req, cutter_msgs::GetWayPoints::Response &res);
    bool readWayPointsFromFile(std::string filename);

    ros::NodeHandle nh_;
    ros::ServiceServer wp_srv_;
    int totalPoints_, currentPoint_;
    std::vector<cutter_msgs::WayPoint> path_;
};


bool CutterPlanner::getWayPointService(
    cutter_msgs::GetWayPoints::Request &req, cutter_msgs::GetWayPoints::Response &res)
{
  if (req.increment)
  {
    if (currentPoint_ < totalPoints_)
      currentPoint_++;
  }
  
  res.pointsLeft = totalPoints_ - currentPoint_;
  
  if (currentPoint_ < totalPoints_)
    res.currWayPoint = path_[currentPoint_];
  else
    res.currWayPoint = cutter_msgs::WayPoint();

  if (currentPoint_ + 1 < totalPoints_)
    res.nextWayPoint = path_[currentPoint_ + 1];
  else
    res.nextWayPoint = cutter_msgs::WayPoint();

  ROS_INFO("Request: continue: %d, totalPoints: %d, currentPoint: %d, pointsLeft: %d", req.increment, totalPoints_, currentPoint_, res.pointsLeft);
  return true;
}


bool CutterPlanner::readWayPointsFromFile(std::string filename)
{
  cutter_msgs::WayPoint tempWP;
  tempWP.pose.position.x = 10;//0.5;
  tempWP.pose.position.y = 10;//-5.0;
  tempWP.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  tempWP.distanceTol = .1;
  tempWP.direction = false;
  path_.push_back(tempWP);

  tempWP.pose.position.x = 10;//0.5;
  tempWP.pose.position.y = 20;//5.0;
  tempWP.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  tempWP.distanceTol = .1;
  tempWP.direction = false;
  path_.push_back(tempWP);

  tempWP.pose.position.x = 0;//-0.5;
  tempWP.pose.position.y = 20;//5.0;
  tempWP.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  tempWP.distanceTol = .1;
  tempWP.direction = false;
  path_.push_back(tempWP);

  tempWP.pose.position.x = 0;//;
  tempWP.pose.position.y = 10;//-5.0;
  tempWP.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  tempWP.distanceTol = .1;
  tempWP.direction = false;
  path_.push_back(tempWP);

  totalPoints_ = path_.size();
  return true;
}


//##constructor##
CutterPlanner::CutterPlanner()
{
  currentPoint_ = 0;


  readWayPointsFromFile("yomama");

  ROS_INFO("Advertising service");
  wp_srv_ = nh_.advertiseService("get_waypoint", &CutterPlanner::getWayPointService, this );

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_waypoint_server");

  CutterPlanner planner;

  ROS_INFO("Ready to serve up waypoints.");

  ros::spin();

  return 0;
}
