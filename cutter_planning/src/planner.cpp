// CWRU cutter path planner

#include <ros/ros.h>
#include "cutter_msgs/GetWayPoints.h"
#include <tf/tf.h>
#include <string>
#include <iostream>
#include <fstream>
#include <ros/package.h>
#include <ros/console.h>
using namespace std;

class CutterPlanner
{
  public:
    CutterPlanner();

  private:
    bool getWayPointService(cutter_msgs::GetWayPoints::Request &req, cutter_msgs::GetWayPoints::Response &res);
    bool readWayPointsFromFile(std::string filename);
    void parseStringFormat(std::string line, cutter_msgs::WayPoint &tempWP);

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

void CutterPlanner::parseStringFormat(std::string line, cutter_msgs::WayPoint &tempWP){
      int lastcomma;
      //ROS_INFO("Waypoint coord 1: %f",strtod(line.substr(line.find_first_of("(")+1,line.find_first_of(",")-1).c_str(),NULL));
      tempWP.pose.position.x = strtod(line.substr(line.find_first_of("(")+1,line.find_first_of(",")-1).c_str(),NULL);
      //ROS_INFO("Waypoint coord 1: %f",strtod(line.substr(line.find_first_of("(")+1,line.find_first_of(",")-1).c_str(),NULL));
      tempWP.pose.position.y = strtod(line.substr(line.find_first_of(",")+1,line.find_first_of(",",line.find_first_of(",")+1)-1).c_str(), NULL);
      lastcomma = line.find_first_of(",",line.find_first_of(",")+1)+1;
      tempWP.pose.orientation = tf::createQuaternionMsgFromYaw(strtod(line.substr(lastcomma, line.find_first_of(",", lastcomma)-lastcomma).c_str(), NULL));
      lastcomma = line.find_first_of(",",lastcomma + 1)+1;
      tempWP.distanceTol = strtod(line.substr(lastcomma, line.find_first_of(",", lastcomma)-lastcomma).c_str(), NULL);
      ROS_INFO("WAYPOINT DTOL VALUE IS %f", tempWP.distanceTol);
      tempWP.direction = (strtod(line.substr(line.find_first_of(")")-1, 1).c_str(), NULL) == 1);
}
bool CutterPlanner::readWayPointsFromFile(std::string filename)
{
ROS_INFO("Got path %s", filename.c_str());
  cutter_msgs::WayPoint tempWP;
  const char * name = filename.c_str();
  std::fstream points_list;
  points_list.open(name);
  if(!(points_list.is_open())){//Check if the file exists.
      ROS_INFO("Could not open waypoint file %s.",name);
      return false;
  }
  std::string line;
  vector<string> nums;
  while (true){
      getline(points_list,line);//Read in a line (i.e. a waypoint)
      if(points_list.eof()){
        break;//If we are out of waypoints, end the loop.
      }//For each waypoint...
      CutterPlanner::parseStringFormat( line, tempWP);//Parse the waypoint's string-based format into usable numbers.
      path_.push_back(tempWP);
  }
  points_list.close();
  totalPoints_ = path_.size();
  return true;
}


//##constructor##
CutterPlanner::CutterPlanner()
{
  currentPoint_ = 0;
  ros::NodeHandle nh;
  std:string file;
  nh.getParam("file", file);
  ROS_INFO("Got file %s", file.c_str());
  std::string path = ros::package::getPath("cutter_planning");

  
  
  readWayPointsFromFile(path + "/WP/" + file);

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
