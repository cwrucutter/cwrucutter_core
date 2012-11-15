/* Copyright (c) 2012, EJ Kreinar
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 ********************************************************************************
 * convert_gps_to_pose.cpp
 *   Converts a GPS NavSatFix message to a PoseWithCovarianceStamped
 *   (But, I could also publish a PoseStamped which can be visualized in Rviz)
 * 
 * Subscribes:
 *   - gps_fix (sensor_msgs/NavSatFix): Current GPS reading
 *   - odom (nav_msgs/Odometry): Odometry measurement
 *  
 * Publishes:
 *   - state (cutter_msgs/State): Current state of the robot in the /map frame
 *
 * Broadcasts:
 *   - map->odom transform: Odometry correction
 *
 * Parameters:
 *   - ref_lat: Latitude reference point
 *   - ref_lon: Longitude reference point
 *   - rel_alt: Altitude reference point
 *   - ~gps: Topic name of the subscribed NavSatFix message
 *   - ~pose: Topic name of the published PoseWithCovarianceStamped message
 *
 ********************************************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <math.h>
#include "angles/angles.h"
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include "GPS_conversion.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class CutterGPSConversion
{
  public:
    CutterGPSConversion();

  private:
    void gpsCallback(const sensor_msgs::NavSatFix& gps);
    void velCallback(const geometry_msgs::TwistStamped& vel);
    void publishState();
    bool checkForMessages();

    sensor_msgs::NavSatFix fix_;
    geometry_msgs::TwistStamped vel_;
    bool fix_rcv_;
    bool vel_rcv_;
  
    ros::NodeHandle node_;
    ros::Publisher  pose_pub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber vel_sub_;
 
    sensor_msgs::NavSatFix ref_;
};

CutterGPSConversion::CutterGPSConversion()
{
  // Get parameters
  double lat, lon, alt;
  if (!(node_.getParam("ref_lat",lat) &&
        node_.getParam("ref_lon",lon) &&
        node_.getParam("ref_alt",alt)))
  {
    // Provide a default reference point if there's no reference params
    lat = 41.5012408976;
    lon = -81.6063695197;
    alt = 216;
    ROS_WARN("Reference parameters not found (ref_lat, ref_lon, ref_alt). Using defaults");
  }
  ref_.latitude  = lat;
  ref_.longitude = lon;
  ref_.altitude  = alt;

  // Get topic names to subscribe/publish
  std::string gps, pose, vel;
  if (!(ros::param::get("~gps",gps) &&
        ros::param::get("~pose",pose) &&
        ros::param::get("~vel",vel) ))
  {
    gps  = "gps_fix";
    pose = "gps_pose"; 
    vel  = "gps_vel";
    ROS_WARN("Reference parameters not found (~gps, ~pose, ~vel). Using defaults");
  }
  
  // Setup Publishers and subscribers
  pose_pub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(pose,1);
  gps_sub_ = node_.subscribe(gps, 1, &CutterGPSConversion::gpsCallback, this);
  vel_sub_ = node_.subscribe(vel, 1, &CutterGPSConversion::velCallback, this);
}

void CutterGPSConversion::publishState()
{
  fix_rcv_ = false;
  vel_rcv_ = false;
  
  geometry_msgs::Point point;
  GPS_conversion::LLA2ENU(fix_, ref_, point);

  ros::Time current_time = ros::Time::now();  

  // Pose message: Setup header
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = fix_.header.stamp; //TODO: Should I use the current time or use the time from the gps message??
  pose_msg.header.frame_id = "map";

  // Pose message: Set the pose
  pose_msg.pose.pose.position = point;

  // Pose message: Set the orientation
  double trackAngle = atan2(vel_.twist.linear.y, vel_.twist.linear.x);
  //ROS_INFO("Xvel: %f, Yvel: %f, angle: %f", vel_.twist.linear.x, vel_.twist.linear.y, trackAngle);
  pose_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(trackAngle);

  // Pose message: Set the covariance
  // TODO: Set the covariance according to the NavSatFix message
  // spose_msg.pose.covariance = ....

  // Publish the state
  pose_pub_.publish(pose_msg);
  ROS_INFO("Published pose");
}

void CutterGPSConversion::gpsCallback(const sensor_msgs::NavSatFix& gps)
{
  fix_ = gps;
  fix_rcv_ = true;
  //ROS_INFO("Received gps fix");
  if (checkForMessages())
    publishState();
}

void CutterGPSConversion::velCallback(const geometry_msgs::TwistStamped& vel)
{
  vel_ = vel;
  vel_rcv_ = true;
  //ROS_INFO("Received gps velocity");
  if (checkForMessages())
    publishState();
}

bool CutterGPSConversion::checkForMessages()
{
  if (fix_rcv_ && vel_rcv_) // Check if we have both messages
    return (fix_.header.stamp - vel_.header.stamp) < ros::Duration(.5); // Compare the time stamps
  else
    return false;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization");
  CutterGPSConversion conversion;
  ros::spin();
/*
  while (ros::ok())
  {
    if (conversion.fix_rcv_ && conversion.vel_rcv_)
    {
      if ( (conversion.fix_.header.stamp - conversion.vel_.header.stamp) < ros::Duration(.5) )
      {
        ROS_INFO("WHOOOOOOOOOOOOOOOOOOOOOO!!!!!");
        conversion.fix_rcv_ = false;
        conversion.vel_rcv_ = false;
      }
    }
    ros::spinOnce();  
  }
*/
  return 0;
}
