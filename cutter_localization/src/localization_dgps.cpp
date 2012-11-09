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
 * localization_dgps.cpp
 *   Localizes using GPS measurements  
 *  
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
#include <wrappers/matrix/matrix_wrapper.h>

using namespace MatrixWrapper;

class LocalizationDGPS
{
  public:
    LocalizationDGPS();

  private:
    void dgpsCallback(const sensor_msgs::NavSatFix& gps);
    void odomCallback(const nav_msgs::Odometry& odom);
    void publishState();
    void KalmanFilterUpdate();

    Matrix x_;
    Matrix P_;

    nav_msgs::Odometry odom_;
    nav_msgs::Odometry odom_last_;
    geometry_msgs::Pose meas_;

    ros::NodeHandle node_;
    ros::Publisher  state_pub_;
    ros::Subscriber dgps_sub_;
    ros::Subscriber odom_sub_;
    tf::TransformBroadcaster tf_br_;
    tf::StampedTransform tf_map_to_odom_;
 
    sensor_msgs::NavSatFix ref_;
    bool odom_rcv_;
    bool gps_rcv_;

    double track_;
    double sigma_enc_;
};

LocalizationDGPS::LocalizationDGPS()
{
  // Get parameters
  double lat, lon, alt;
  if (!(ros::param::get("~ref_lat",lat) &&
       ros::param::get("~ref_lon",lon) &&
       ros::param::get("~ref_alt",alt)))
  {
    lat = 41.5012408976;
    lon = -81.6063695197;
    alt = 216;
  }
  ref_.latitude  = lat;
  ref_.longitude = lon;
  ref_.altitude  = alt;
  
  track_ = .59; //TODO: Change this to a param
  sigma_enc_ = .001;
  
  // Setup Publishers and subscribers
  state_pub_ = node_.advertise<cutter_msgs::State>("state",1);
  dgps_sub_  = node_.subscribe("gps_fix", 1, &LocalizationDGPS::dgpsCallback, this);
  odom_sub_  = node_.subscribe("odom", 10, &LocalizationDGPS::odomCallback, this);

  // Setup TF frame conversion
  tf_map_to_odom_.frame_id_ = std::string("map");
  tf_map_to_odom_.child_frame_id_ = std::string("odom");

  odom_rcv_ = false;
  gps_rcv_  = false;

  // Setup matrices
  x_ = Matrix(3,1);
  x_ = 0;
  P_ = Matrix(3,3);
  P_(1,1) = 99999; P_(2,2) = 99999; P_(3,3) = 99999;
   
}

void LocalizationDGPS::odomCallback(const nav_msgs::Odometry& odom)
{
  odom_ = odom;
  odom_rcv_ = true;
  ROS_INFO("Received odometry");
}

void LocalizationDGPS::dgpsCallback(const sensor_msgs::NavSatFix& gps)
{
  GPS_conversion::LLA2ENU(gps, ref_, meas_);
  gps_rcv_ = true;
  ROS_INFO("Received gps");
}

void LocalizationDGPS::KalmanFilterUpdate()
{
  // Extended Kalman Filter based on:
  // http://kom.aau.dk/group/05gr999/reference_material/filtering/eKf-3state.pdf

  //Generate the x_pre estimate
  double xdiff = odom_.pose.pose.position.x - odom_last_.pose.pose.position.x;
  double ydiff = odom_.pose.pose.position.y - odom_last_.pose.pose.position.y;
  double thtdiff = tf::getYaw(odom_.pose.pose.orientation) - tf::getYaw(odom_last_.pose.pose.orientation);
  double mag_d = sqrt(xdiff*xdiff + ydiff*ydiff);
  double thtmid = x_(3,1)+thtdiff/2;
  Matrix x_pre = x_;
  x_pre(1,1) = x_pre(1,1) + xdiff;
  x_pre(2,1) = x_pre(2,1) + ydiff;
  x_pre(3,1) = x_pre(3,1) + thtdiff;

  // Generate the P_pre estimate
  Matrix Ak(3,3); Ak = 0;
  Ak(1,1) = 1;  Ak(2,2) = 1;
  Ak(1,3) = -mag_d * sin(thtmid);
  Ak(2,3) = -mag_d * cos(thtmid);
  Ak(3,3) = 1;
  Matrix Bk(3,2); Bk = 0;
  Bk(1,1) = .5*cos(thtmid)-mag_d/2/track_*sin(thtmid);
  Bk(1,2) = .5*cos(thtmid)+mag_d/2/track_*sin(thtmid);
  Bk(2,1) = .5*sin(thtmid)+mag_d/2/track_*cos(thtmid);
  Bk(2,2) = .5*sin(thtmid)-mag_d/2/track_*cos(thtmid);
  Bk(3,1) = 1/track_;
  Bk(3,2) = -1/track_;
  Matrix P_pre(3,3);
  Matrix Qk(3,3); Qk = 0;
  Qk(1,1) = .01*.01;
  Qk(2,2) = .01*.01;
  Qk(3,3) = .01*.01;
  P_pre = Ak*P_*Ak.transpose() + Bk*Bk.transpose()*sigma_enc_*sigma_enc_ + Qk;

  
   
  
}

void LocalizationDGPS::publishState()
{
  ros::Time current_time = ros::Time::now();  

  //TODO Redo

/*
  // State
  cutter_msgs::State state;
  state.header.stamp = current_time;  //Populate header
  state.header.frame_id = "map";

  // For now, set the state's pose to the GPS pose, and 
  // the twist to the odometry's velocity
  state.pose.pose = meas_;
  state.twist = odom_.twist;

  // Transform broadcaster
  tf_map_to_odom_.stamp_ = current_time;
  tf_map_to_odom_.setOrigin(tf::Vector3(state.pose.pose.position.x - odom_.pose.pose.position.x, 
                                        state.pose.pose.position.y - odom_.pose.pose.position.y, 0 ) );
  tf_map_to_odom_.setRotation(tf::createQuaternionFromYaw(0));

  // Publish the state
  state_pub_.publish(state);

  // Broadcast the transform
  tf_br_.sendTransform(tf_map_to_odom_);
*/
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
