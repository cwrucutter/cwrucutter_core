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
 * slip_detect.cpp
 *   Fuses odometry and IMU to detect if there has been a wheel slip
 * 
 * Subscribes:
 *   - odom (nav_msgs/Odometry): wheel odometry
 *   - imu/data (sensor_msgs/Imu): IMU output from christa IMU
 *  
 * Publishes:
 *
 ********************************************************************************/

#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "math.h"
#include "cutter_msgs/Testing.h"
#include "cutter_msgs/SlipStatus.h"
#include <vector>
#include <algorithm>
#include "kalman_velocity.h"

// RunningStat class source code from the helpful website:
// http://www.johndcook.com/standard_deviation.html
// Thanks!
class RunningStat 
{
public:
  RunningStat() : m_n(0) {}
  
  void Clear()
  {
    m_n = 0;
  }

  void Push(double x)
  {
    m_n++;

    // See Knuth TAOCP vol 2, 3rd edition, page 232
    if (m_n == 1)
    {
      m_oldM = m_newM = x;
      m_oldS = 0.0;
    }
    else
    {
      m_newM = m_oldM + (x - m_oldM)/m_n;
      m_newS = m_oldS + (x - m_oldM)*(x - m_newM);
    
      // set up for next iteration
      m_oldM = m_newM; 
      m_oldS = m_newS;
    }
  }

  int NumDataValues() const
  {
    return m_n;
  }

  double Mean() const
  {
    return (m_n > 0) ? m_newM : 0.0;
  }

  double Variance() const
  {
    return ( (m_n > 1) ? m_newS/(m_n - 1) : 0.0 );
  }

  double StandardDeviation() const
  {
    return sqrt( Variance() );
  }

private:
  int m_n;
  double m_oldM, m_newM, m_oldS, m_newS;
};

class SlipDetect
{
public:
  SlipDetect();
  void filter();
  bool initialize();
  bool lookupParams();
  void setLoopRate(double rate) { loop_rate_ = rate; };

private:
  // Ros Classes
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher test_pub_;
  ros::Publisher slip_pub_;
  ros::NodeHandle nh_;
  
  RunningStat imu_stat_x_;
  RunningStat imu_stat_y_;

  // Kalman Filter parameters
  double odom_var_v_;
  double odom_var_w_;
  double imu_var_a_;
  double imu_var_w_;
  double proc_var_v_;
  double proc_var_w_;
  double proc_var_a_;
  double proc_var_wdot_;
  
  KalmanVelocity central_filter_;
  KalmanVelocity kf_enc_;
  KalmanVelocity kf_imu_;

  double loop_rate_;
 
  // Store odom and imu values from callbacks
  nav_msgs::Odometry last_odom_;
  sensor_msgs::Imu   last_imu_;
  bool imu_new_;
  bool odom_new_;
  
  // IMU Initialization
  bool initialized_;
  int initialized_cnt_;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
  {
    last_odom_ = *odom;
    odom_new_ = true;
  };

  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
  {
    last_imu_ = *imu;
    imu_new_ = true;
  };

};

SlipDetect::SlipDetect()
{
  loop_rate_ = 0;
  initialized_cnt_ = 0;
  initialized_ = false;
  odom_new_ = false;
  imu_new_  = false;
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom",10,&SlipDetect::odomCallback,this);
  imu_sub_  = nh_.subscribe<sensor_msgs::Imu>("imu/data",10,&SlipDetect::imuCallback,this);
  test_pub_ = nh_.advertise<cutter_msgs::Testing>("cwru/testing",1);
  slip_pub_ = nh_.advertise<cutter_msgs::SlipStatus>("cwru/slip",1);
};

bool SlipDetect::lookupParams()
{
  
  bool rval = ros::param::get("~odom_var_v", odom_var_v_)
           && ros::param::get("~odom_var_w", odom_var_w_)
           && ros::param::get("~imu_var_a",  imu_var_a_)
           && ros::param::get("~imu_var_w",  imu_var_w_)
           && ros::param::get("~proc_var_v",  proc_var_v_)
           && ros::param::get("~proc_var_w",  proc_var_w_)
           && ros::param::get("~proc_var_a",  proc_var_a_)
           && ros::param::get("~proc_var_wdot",  proc_var_wdot_);
    
  return rval;
}

bool SlipDetect::initialize()
{
  
  if (loop_rate_ == 0)
  {
    ROS_ERROR("Loop rate not specified. Use setLoopRate(double rate) before initializing");
    return false
  }
  
  bool rval = true;
  rval &= central_filter_.initialize(1/loop_rate_, proc_var_v_, proc_var_w_, proc_var_a_, proc_var_wdot_);
  rval &= central_filter_.initializeEncoderNoise(odom_var_v_, odom_var_w_);
  rval &= central_filter_.initializeIMUNoise(imu_var_a_, imu_var_w_);
  
  rval &= kf_enc_.initialize(1/loop_rate_, proc_var_v_, proc_var_w_, proc_var_a_, proc_var_wdot_);
  rval &= kf_enc_.initializeEncoderNoise(odom_var_v_, odom_var_w_);
  
  rval &= kf_imu_.initialize(1/loop_rate_, proc_var_v_, proc_var_w_, proc_var_a_, proc_var_wdot_);
  rval &= kf_imu_.initializeIMUNoise(imu_var_a_, imu_var_w_);
  
  return rval;
}

void SlipDetect::filter()
{  
  double odom_v, odom_w, imu_x, imu_y, imu_w;
  odom_v = last_odom_.twist.twist.linear.x;
  odom_w = last_odom_.twist.twist.angular.z;
  imu_x  = last_imu_.linear_acceleration.x;
  imu_y  = last_imu_.linear_acceleration.y;
  imu_w  = last_imu_.angular_velocity.z * M_PI / 180;
  
  double x_accel, y_accel;
  x_accel = imu_x - imu_stat_x_.Mean();
  y_accel = imu_y - imu_stat_y_.Mean();
  
  cutter_msgs::Testing test;
  cutter_msgs::SlipStatus slip;

  if (imu_new_ && !initialized_)
  {
    if (fabs(odom_v) < .001 && fabs(odom_w) < .001)
    {
      // Filter the IMU when the wheels report no motion (assume the robot is not moving) 
      imu_stat_x_.Push(imu_x);
      imu_stat_y_.Push(imu_y);
      ROS_INFO("Imu X: %f, X average: %f, Stddev: %f, X-avg: %f", 
                    imu_x,  imu_stat_x_.Mean(), imu_stat_x_.StandardDeviation(), imu_x-imu_stat_x_.Mean());
      ROS_INFO("Imu Y: %f, Y average: %f, Stddev: %f, Y-avg: %f", 
                    imu_y,  imu_stat_y_.Mean(), imu_stat_y_.StandardDeviation(), imu_y-imu_stat_y_.Mean());
      ROS_INFO("Odom V: %f, Odom W: %f, fabs(odomv): %f, fabs(odomw): %f", odom_v, odom_w, fabs(odom_v), fabs(odom_w));
      if (initialized_cnt_++ > 50)
        initialized_ = true;
      ROS_INFO("Count: %i", initialized_cnt_);
    }
    else
    {
      // Currently do nothing with the IMU if we're moving??
      ROS_INFO("imu: No motion");
    }
    
  } 
  
  bool no_update = true;
  if (odom_new_ && imu_new_)
  {
    if (initialized_) // Check that IMU has initialized!
    {
      no_update = false;
      
      //central_filter_.addMeasurementEncoders(odom_v, odom_w);
      //central_filter_.addMeasurementIMU(x_accel, imu_w);
      //central_filter_.update();
      
      // Update the Encoder Filter
      kf_enc_.addMeasurementEncoders(odom_v, odom_w);
      kf_enc_.update();
      
      // Update the Auxillary Filter
      kf_imu_.addMeasurementIMU(x_accel, imu_w);
      kf_imu_.update();
      
      // Fuse the two Filters
      double Ptot_v, Ptot_w, Ptot_a, Ptot_wdot;
      double Xtot_v, Xtot_w, Xtot_a, Xtot_wdot;
      Ptot_v    = kf_enc_.cov_[0]  * kf_imu_.cov_[0]  / (kf_enc_.cov_[0]  + kf_imu_.cov_[0]);
      Ptot_w    = kf_enc_.cov_[5]  * kf_imu_.cov_[5]  / (kf_enc_.cov_[5]  + kf_imu_.cov_[5]);
      Ptot_a    = kf_enc_.cov_[10] * kf_imu_.cov_[10] / (kf_enc_.cov_[10] + kf_imu_.cov_[10]);
      Ptot_wdot = kf_enc_.cov_[15] * kf_imu_.cov_[15] / (kf_enc_.cov_[15] + kf_imu_.cov_[15]);
      
      Xtot_v    = Ptot_v / kf_enc_.cov_[0]  * kf_enc_.state_[0]  +  Ptot_v / kf_imu_.cov_[0]  * kf_imu_.state_[0];
      Xtot_w    = Ptot_w / kf_enc_.cov_[5]  * kf_enc_.state_[1]  +  Ptot_w / kf_imu_.cov_[5]  * kf_imu_.state_[1];
      Xtot_a    = Ptot_a / kf_enc_.cov_[10] * kf_enc_.state_[2]  +  Ptot_a / kf_imu_.cov_[10] * kf_imu_.state_[2];
      Xtot_wdot = Ptot_wdot / kf_enc_.cov_[15] * kf_enc_.state_[3]  +  Ptot_wdot / kf_imu_.cov_[15] * kf_imu_.state_[3];
      
      ROS_INFO("Enc Cov Diag: [ %f, %f, %f, %f]", kf_enc_.cov_[0], kf_enc_.cov_[5], kf_enc_.cov_[10], kf_enc_.cov_[15]);
      ROS_INFO("Imu Cov Diag: [ %f, %f, %f, %f]", kf_imu_.cov_[0], kf_imu_.cov_[5], kf_imu_.cov_[10], kf_imu_.cov_[15]);
      ROS_INFO("Tot Cov Diag: [ %f, %f, %f, %f]", Ptot_v, Ptot_w, Ptot_a, Ptot_wdot);
      
      // Save off all the debugging info I want
      test.corrected_imu_x = x_accel;
      test.corrected_imu_y = y_accel;
      test.kf_v = Xtot_v;
      test.kf_w = Xtot_w;
      test.kf_a = Xtot_a;
      test.kf_wdot = Xtot_wdot;
      test.innov_v = Xtot_v - odom_v;
      test.innov_w = Xtot_w - odom_w;
      test.innov_imu_a = Xtot_a - x_accel;
      test.innov_imu_w = Xtot_w - imu_w;
      test.innov_bound_v = 3*(sqrt(Ptot_v) + odom_var_v_); 
      test.innov_bound_w = 3*(sqrt(Ptot_w) + odom_var_w_); 
      test.innov_bound_imu_a = 3*(sqrt(Ptot_a) + imu_var_a_); 
      test.innov_bound_imu_w = 3*(sqrt(Ptot_w) + imu_var_w_); 
       
      // Calculate the certainty for each measurement:
      //    - certainty = measurement / sqrt(covariance)
      // if meas > 3*sqrt(cov), then there's like a 1% chance or less that 
      // the measurement is correct
      double slip_enc_v, slip_enc_w, slip_gyro, slip_accel = 0;
      if ( (sqrt(Ptot_v) + odom_var_v_) > .01)
        slip_enc_v = fabs(Xtot_v - odom_v) / (sqrt(Ptot_v) + odom_var_v_);
      if ( (sqrt(Ptot_w) + odom_var_w_) > .01)
        slip_enc_w = fabs(Xtot_w - odom_w) / (sqrt(Ptot_w) + odom_var_w_);
      if ( (sqrt(Ptot_w) + imu_var_w_) > .01)
        slip_gyro  = fabs(Xtot_w - imu_w) / (sqrt(Ptot_w) + imu_var_w_) ;
      if ( (sqrt(Ptot_a) + imu_var_a_) > .01)
        slip_accel = fabs(Xtot_a - x_accel) / (sqrt(Ptot_a) + imu_var_a_) ;
      
      double max_innov = std::max(std::max(std::max(slip_enc_v,slip_enc_w),slip_gyro),slip_accel);
      slip.slip_enc_v = slip_enc_v;
      slip.slip_enc_w = slip_enc_w;
      slip.slip_gyro  = slip_gyro;
      slip.slip_accel = slip_accel;
      slip.slip_max = max_innov;
      if (max_innov > 3) // Slip if the max innovation is > 3
        slip.slip = 1;
      else
        slip.slip = 0;
        
      // TODO: If we slip, maybe take the other filter out of the fused measurement??
      
      // TODO: Implement an "integral term" on the slip.. If we're consistently have a larger
      //       than expected innovation, this could indicate a slip
      
      // TODO: Have this node publish the filtered odometry?? 
      
      slip_pub_.publish(slip);
    }
  }
  
  imu_new_ = false;
  odom_new_ = false;

  if (no_update)
  {
    // Populate the kalman filter variables with the last state
    test.corrected_imu_x = x_accel;
    test.corrected_imu_y = y_accel;
    test.kf_v = central_filter_.state_[0];
    test.kf_w = central_filter_.state_[1];
    test.kf_a = central_filter_.state_[2];
    test.kf_wdot = central_filter_.state_[3];
  }

  test_pub_.publish(test);
  
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "slip_detect"); //Init ROS
  SlipDetect detector; //Construct class

  double looprate = 20;
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
