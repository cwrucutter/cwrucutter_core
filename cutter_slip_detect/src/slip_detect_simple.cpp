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
 * slip_detect_simple.cpp
 *   Checks if the GPS forward velocity agrees with the encoder forward velocity
 * 
 * Subscribes:
 *   - odom (nav_msgs/Odometry): wheel odometry
 *   - imu/data (sensor_msgs/Imu): IMU output from christa IMU
 *   - gps_pose (geometry_msgs/PoseStamped): gps output in map frame
 *  
 * Publishes:
 *   - cwru/slip
 *
 ********************************************************************************/

#include "ros/ros.h"
#include "math.h"
#include <vector>
#include <algorithm>
#include "kalman_velocity.h"

// Messages
#include "cutter_msgs/Testing.h"
#include "cutter_msgs/SlipStatus.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"

// Transform stuff
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "message_filters/subscriber.h"

class MovingAverage 
{
public:
  MovingAverage() : m_n(0), m_index(0), m_vals(0) {}
  
  void Clear()
  {
    m_n = 0;
    m_index = 0;
    m_vals.clear();
    m_vals.resize(0);
  }
  
  void SetWindow(int window)
  {
    m_n_window = window;
  }

  void Push(double x)
  {
    if (m_n < m_n_window)
    {
      m_n++;                // Increment number of elements
      m_vals.push_back(x);  // Add a new element
      m_index++;            // Increment index
    }
    else
    {
      m_vals[m_index++] = x; // Overwrite old element at index with the new element
    }
    
    if (m_index >= m_n_window) // Check if index is greater than the window size
      m_index = 0;
  }

  int NumDataValues() const
  {
    return m_n;
  }

  double Mean()
  {
    if (m_n == m_n_window)
    {
      double total = 0;
      std::vector<double>::iterator it;
      for (it = m_vals.begin(); it != m_vals.end(); it++)
      {
        total += *it;
      }
      m_mean = total / m_n_window;
    }
    return (m_n == m_n_window) ? m_mean : 0.0;
  }

private:
  int m_n_window; // Size of window
  int m_n;        // Current total # of elements in filter
  int m_index;
  std::vector<double> m_vals;
  double m_mean;
};

class SlipDetectSimple
{
public:
  SlipDetectSimple();
  void filter();
  bool initialize();
  bool lookupParams();
  void setLoopRate(double rate) { loop_rate_ = rate; };

private:
  void getGPSLeverarm();
  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
  void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& gps);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);

  // Ros Classes
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber gps_sub_;
  ros::Publisher slip_pub_;
  ros::Publisher test_pub_;
  ros::NodeHandle nh_;
  tf::TransformListener listener;
  
  // Moving average Filters for GPS and Encoders
  MovingAverage gps_avg_window_;
  MovingAverage enc_avg_window_;

  // Filter Parameters
  double avg_window_;
  double slip_threshold_;
  double gps_leverarm_;

  double loop_rate_;
 
  // Store odom and gps values from callbacks
  nav_msgs::Odometry odom_;
  geometry_msgs::Point old_gps_;
  geometry_msgs::Point gps_;
  double gps_vel_;
  double enc_vel_;
  double imu_w_;
  bool odom_new_;
  bool gps_new_;
  bool leverarm_valid_;
  
  // IMU Initialization
  bool initialized_;
  int initialized_cnt_;
};

SlipDetectSimple::SlipDetectSimple()
{
  loop_rate_ = 0;
  initialized_cnt_ = 0;
  initialized_ = false;
  odom_new_ = false;
  gps_new_  = false;
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom",10,&SlipDetectSimple::odomCallback,this);
  imu_sub_  = nh_.subscribe<sensor_msgs::Imu>("imu/data",10,&SlipDetectSimple::imuCallback,this);
  gps_sub_  = nh_.subscribe<geometry_msgs::PoseStamped>("gps_pose",10,&SlipDetectSimple::gpsCallback,this);
  test_pub_ = nh_.advertise<cutter_msgs::Testing>("cwru/testing",1);
  slip_pub_ = nh_.advertise<cutter_msgs::SlipStatus>("cwru/slip",1);
};

bool SlipDetectSimple::lookupParams()
{
  
  bool rval = ros::param::get("~avg_window", avg_window_)
           && ros::param::get("~slip_threshold",  slip_threshold_);
    
  return rval;
}

bool SlipDetectSimple::initialize()
{
  
  if (loop_rate_ == 0)
  {
    ROS_ERROR("Loop rate not specified. Use setLoopRate(double rate) before initializing");
    return false;
  }
  
  gps_avg_window_.SetWindow(avg_window_*10); // GPS @ 10 Hz
  enc_avg_window_.SetWindow(avg_window_*25); // Encoders @ 25 Hz
  
  return true;
}

void SlipDetectSimple::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
{
  // Store odometry message
  odom_ = *odom;
  odom_new_ = true;
  
  // Pull off and filter the encoder forward velocity
  enc_vel_ = odom_.twist.twist.linear.x;
  enc_avg_window_.Push(enc_vel_);
}

void SlipDetectSimple::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
  // Pull off the imu angular velocity (I dont care about the rest of the message)
  imu_w_ = imu->angular_velocity.z * M_PI / 180;
};
  
void SlipDetectSimple::gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& gps)
{
  // Store GPS Message
  old_gps_ = gps_;
  
  // Pull off the Last position
  gps_ = gps->pose.position;
  gps_new_ = true;
  
  // Check for the leverarm transform
  if (!leverarm_valid_)
  {
    getGPSLeverarm();     
  }
  else
  {
    double xoff, yoff, gps_dist, odom_v, gps_vel, gps_vel_fix;
    odom_v = odom_.twist.twist.linear.x;
    xoff = gps_.x - old_gps_.x;
    yoff = gps_.y - old_gps_.y;
    
    // GpsDist = new gps - old gps
    gps_dist = sqrt( xoff*xoff + yoff*yoff );
    
    // GpsVel  = GpsDist / 0.1   <--- Distance / GPS DT where GPS DT = 10 Hz or 0.1 s
    gps_vel  = gps_dist / 0.1;
    
    // GpsVelFix = sqrt( GpsVel^2 - (r*w)^2 )   <--- Ie, subtract out component of motion due to angular velocity
    gps_vel_fix = sqrt( gps_vel*gps_vel  -  gps_leverarm_*gps_leverarm_*imu_w_*imu_w_ );
    if (odom_v < 0)
      gps_vel_fix = - gps_vel_fix;
    //ROS_INFO("GPS Velocity: %f, Odom Velocity: %f", gps_vel_fix, odom_v);
    
    // Now pull off and filter the GPS Velocity
    gps_vel_ = gps_vel_fix;
    if (fabs(gps_vel_) < 2.0) // Discard outliers (Robot will never move faster than 2.0 m/s, ever)
    {
      gps_avg_window_.Push(gps_vel_);
    }
    else
    {
      ROS_WARN("GPS Velocity Outlier. Skipping point. Velocity measured: %f", gps_vel_);
    }
  }
}
void SlipDetectSimple::getGPSLeverarm()
{
  tf::StampedTransform transform;
  try{
    listener.waitForTransform("/base_link", "/base_gps", ros::Time(0), ros::Duration(0.1) );
    listener.lookupTransform("/base_link", "/base_gps",
                             ros::Time(0), transform);
    gps_leverarm_ = transform.getOrigin().x();
    leverarm_valid_ = true;
    ROS_INFO("Received gps leverarm: %.3f", gps_leverarm_);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
}

void SlipDetectSimple::filter()
{  
  double enc_fwd_vel, gps_fwd_vel, abs_gps_vel, abs_enc_vel, vel_ratio = 0;
  
  // Average the moving window
  enc_fwd_vel = enc_avg_window_.Mean();
  gps_fwd_vel = gps_avg_window_.Mean();
  
  // Find absolute value
  abs_enc_vel = fabs(enc_fwd_vel);
  abs_gps_vel = fabs(gps_fwd_vel);
  
  cutter_msgs::SlipStatus slip;
  cutter_msgs::Testing test;
  
  // Store debugging data
  test.gpsvel = gps_vel_;
  test.odomvel = enc_vel_;
  test.slip_enc_v = enc_fwd_vel;
  test.slip_gps_v = gps_fwd_vel;

  if (abs_enc_vel > 0.2 && abs_gps_vel > 0.001)
  {
    vel_ratio = abs_enc_vel / abs_gps_vel / slip_threshold_;
  }
  
  slip.slip_max = vel_ratio; 
  test.slip_max = vel_ratio;
  
  // old condition: 
  //if (gps_fwd_vel < 0.1 && enc_fwd_vel > 0.3) //If GPS Velocity < 0.1 AND Encoder velocity > 0.3
  
  // Slip occurs if the Enc:GPS ratio is above 1 (normalized by the slip_threshold)
  //  AND if the GPS velocity is below 0.2 m/s
  slip.slip = 0;
  if (vel_ratio > 1) 
  {
    if (abs_gps_vel < 0.2)
    {
      slip.slip = 1;
      ROS_INFO("STALL Occured! Encoder-to-GPS Ratio: %f", vel_ratio);
    }
    else
    {
      ROS_INFO("Wheels slipping. Robot is still moving. Slip Ratio: %f, GPS Velocity: %f", vel_ratio, gps_fwd_vel);
    }
  }
  
  // TODO: Maybe add other conditions for a slip?
  
  slip_pub_.publish(slip);
  test_pub_.publish(test);

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "slip_detect_simple"); //Init ROS
  SlipDetectSimple detector; //Construct class

  double looprate = 10;
  ros::Rate loop_rate(looprate);
  detector.setLoopRate(looprate);
  if (!detector.lookupParams())
  {
    ROS_ERROR("Parameters not found");
    return 0;
  }
  
  if (!detector.initialize())
  {
    ROS_ERROR("Could not initialize filter");
    return 0;
  }

  while (ros::ok())
  {
    ros::spinOnce();
    detector.filter();
    loop_rate.sleep();
  }
};
