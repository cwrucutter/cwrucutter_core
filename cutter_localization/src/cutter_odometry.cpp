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
 * cutter_odometry.cpp
 *   Listens to odometry sources (encoders, IMU, visual odometry) and outputs
 *   an odometry message which will be used for localization
 * 
 * Subscribes:
 *   - cwru/enc_count (cutter_msgs/EncMsg): Current encoder count from crio
 *   - imu/data (
 *  
 * Publishes:
 *   - odom (nav_msgs/Odometry): Best guess in odometry frame
 *
 * Parameters:
 *   - ~track_width: Robot track width
 *   - ~counts_per_rev_right: Counts per revolution (right wheel)
 *   - ~counts_per_rev_left:  Counts per revolution (left wheel)
 *
 ********************************************************************************/

#include <ros/ros.h>
#include <cutter_msgs/EncMsg.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <math.h>
#include "angles/angles.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"

#define LOOP_RATE 25 //Hz

class CutterOdometry
{
  public:
    CutterOdometry();
    bool lookupParams();
    bool sendOdometry();
    double getLoopRate() {return loop_rate_;}
  
  private:
    void encCountCallback(const cutter_msgs::EncMsg::ConstPtr& enc);  
    void imuCallback(const sensor_msgs::Imu::ConstPtr& enc);    
   
    ros::NodeHandle node_;
    
    ros::Publisher  odom_pub_;
    ros::Subscriber enc_sub_;
    ros::Subscriber imu_sub_;
    tf::TransformBroadcaster odom_broadcaster_;
    
    geometry_msgs::Vector3 imu_angular_rate_;
    
    double x_;
    double y_;
    double tht_;

    double loop_rate_;
    double dt_;
    
    int enc_left_;
    int enc_right_;
    int enc_left_old_;
    int enc_right_old_;

    double track_;
    int ticks_per_m_left_;
    int ticks_per_m_right_;

    bool first_call_;
    bool imu_enabled_;
};

CutterOdometry::CutterOdometry():
  track_(0.67), ticks_per_m_left_(20000), ticks_per_m_right_(20000), loop_rate_(25), imu_enabled_(false)
{
  first_call_ = 1;

  enc_left_  = 0;
  enc_right_ = 0;
  enc_left_old_  = 0;
  enc_right_old_ = 0;
  x_   = 0;
  y_   = 0;
  tht_ = 0;
  dt_  = loop_rate_;

  odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom",1);
  enc_sub_  = node_.subscribe<cutter_msgs::EncMsg>("cwru/enc_count",1,&CutterOdometry::encCountCallback,this);
  imu_sub_  = node_.subscribe<sensor_msgs::Imu>("imu/data",1,&CutterOdometry::imuCallback,this);
}

bool CutterOdometry::sendOdometry()
{
  //ROS_INFO("Sending Odometry: %i, %i, %f, %i, %i",enc_left_,enc_right_,track_,ticks_per_m_right_,ticks_per_m_left_);
  
  double start = ros::Time::now().toSec();

  double Dr, Dl, Vr, Vl, w;
  Dr = double(enc_right_ - enc_right_old_) / ticks_per_m_right_;
  Dl = double(enc_left_  - enc_left_old_ ) / ticks_per_m_left_;
  Vr = Dr / dt_;
  Vl = Dl / dt_;
  w = 0;

  double diff = Vr - Vl;
  double sum  = Vr + Vl;
  double xnew, ynew, thtn;
  xnew = x_; ynew = y_; thtn = tht_;
  if (diff == 0 && sum == 0)
  {
    //Keep the same x,y,theta
  }
  else if (diff < .0000001 && diff > -.0000001)
  {
    // New x,y. Theta stays the same
    xnew = x_ + sum/2 * cos(tht_)*dt_;
    ynew = y_ + sum/2 * sin(tht_)*dt_;
  }
  else
  {
    // New x,y,theta
    if (imu_enabled_)
      w = .1*diff/track_ + .9*imu_angular_rate_.z;
    else
      w = diff/track_;
    //xnew = x_ + track_ * sum/(2*diff) * (sin(diff*dt_/track_+tht_) - sin(tht_));
    //ynew = y_ - track_ * sum/(2*diff) * (cos(diff*dt_/track_+tht_) - cos(tht_));
    xnew = x_ + sum/(2*w) * (sin(w*dt_+tht_) - sin(tht_));
    ynew = y_ - sum/(2*w) * (cos(w*dt_+tht_) - cos(tht_));
    thtn = angles::normalize_angle(tht_ + w*dt_);
  }  

  double end = ros::Time::now().toSec();
  ROS_INFO("Encoders: Dr: %f Dl: %f x_: %f y_: %f tht_: %f",Dr,Dl,x_,y_,tht_);
  ROS_INFO("          xnew: %f ynew: %f thtn: %f duration: %f",xnew,ynew,thtn,end-start);
  ROS_INFO("          Vr: %f Vl: %f", Vr, Vl);
  ROS_INFO("          Loop_rate: %f, DT: %f", loop_rate_, dt_);
  
  // Store Values
  x_   = xnew;
  y_   = ynew;
  tht_ = thtn;
  enc_right_old_ = enc_right_;
  enc_left_old_  = enc_left_;

  // Create a quaternion to store yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(tht_);
  ros::Time current_time = ros::Time::now();
  ros::Time future_time = current_time + ros::Duration(0.1);

  // Broadcast the odom->base_link transform
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster_.sendTransform(
      tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(tht_), tf::Vector3(x_, y_, 0.0)),
      current_time, "odom", "base_link"));
  
  // Publish the Odometry Message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom"; 
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x  = x_;
  odom.pose.pose.position.y  = y_;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x  = sum/2;
  odom.twist.twist.angular.z = w;
  odom_pub_.publish(odom);

  return true;
}

bool CutterOdometry::lookupParams()
{
  bool rval = ros::param::get("~track", track_)
           && ros::param::get("~tpm_left",ticks_per_m_left_)
           && ros::param::get("~tpm_right",ticks_per_m_right_)
           && ros::param::get("~loop_rate",loop_rate_)
           && ros::param::get("~use_imu",imu_enabled_);
  if (rval)
  {
    dt_ = 1/loop_rate_;
  }
  return rval;
}

void CutterOdometry::encCountCallback(const cutter_msgs::EncMsg::ConstPtr& enc)
{
  enc_left_  = enc->left;
  enc_right_ = enc->right;
  if (first_call_)
  {
    enc_right_old_ = enc_right_;
    enc_left_old_  = enc_left_;
    first_call_ = 0;
  }
  ROS_INFO("Received encoder values. Right: %i, Left: %i", enc_right_, enc_left_);
  return;
}

void CutterOdometry::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  imu_angular_rate_ = imu->angular_velocity;
  ROS_INFO("Received imu values. Yaw Rate: %f", imu_angular_rate_.z);
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry");
  CutterOdometry odometry;
  
  //ros::spin();
  if (!odometry.lookupParams())
    ROS_ERROR("Parameters not found");

  ros::Rate loop_rate(odometry.getLoopRate());
  while (ros::ok())
  {
    ros::spinOnce();
    odometry.sendOdometry();
    loop_rate.sleep();
  }

  return 0;
}
