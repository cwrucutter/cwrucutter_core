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
 * localization.cpp
 *   Localizes using GPS measurements  
 *  
 * 
 * Subscribes:
 *   - gps_fix (sensor_msgs/NavSatFix): Current GPS reading
 *  
 * Publishes:
 *   - state (cutter_msgs/State): Current state of the robot in the /map frame
 *
 * Broadcasts:
 *   - map->odom transform: Odometry correction
 *
 * Parameters:
 *   - ~ref_lat: Latitude reference point
 *   - ~ref_lon: Longitude reference point
 *   - ~rel_alt: Altitude reference point
 *
 ********************************************************************************/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <math.h>
#include "angles/angles.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>
#include <cutter_msgs/State.h>
#include "GPS_conversion.h"

class LocalizationDGPS
{
  public:
    LocalizationDGPS();

  private:
    void dgpsCallback(const sensor_msgs::NavSatFix& gps);
    
    ros::NodeHandle node_;
    ros::Publisher  state_pub_;
    ros::Subscriber dgps_sub_;
    
    sensor_msgs::NavSatFix ref_;
};

LocalizationDGPS::LocalizationDGPS()
{
  ref_.latitude  = 41.5012408976;
  ref_.longitude = -81.6063695197;
  ref_.altitude  = 216;

  state_pub_ = node_.advertise<cutter_msgs::State>("state",1);
  dgps_sub_  = node_.subscribe("gps_fix",1,&LocalizationDGPS::dgpsCallback,this);
}

void LocalizationDGPS::dgpsCallback(const sensor_msgs::NavSatFix& gps)
{
  geometry_msgs::Pose local;
  GPS_conversion::LLA2ENU(gps, ref_, local); //Store the converted pose in "local"

  ROS_INFO("GPS converted: X: %f, Y: %f, Z: %f", local.position.x, local.position.y, local.position.z); 
  
  cutter_msgs::State state;
  state.pose.pose = local;

  state_pub_.publish(state); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localization");
  LocalizationDGPS localization;
 
  //ros::spin();

  ros::Rate loop_rate(10.0);
  while (ros::ok())
  {
    if (true /*lookupParams*/)
    {
      ros::spinOnce();
    }
    else
    {
      ROS_WARN("Parameters not found");
    }
    loop_rate.sleep();
  }

  return 0;
}
