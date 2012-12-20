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

class KalmanVelocity
{
// Class for the Kalman Filter on velocity and accelerations, 
// Provides update methods for encoders and IMUs
public: 
  KalmanVelocity(); 
  bool initialize(double dt, double proc_var_v, double proc_var_w, double proc_var_a, double proc_var_wdot);
  bool initializeEncoderNoise(double odom_var_v, double odom_var_w);
  bool initializeIMUNoise(double odom_var_v, double odom_var_w); 
  bool addMeasurementEncoders(double odom_v, double odom_w);
  bool addMeasurementIMU(double imu_a, double imu_w);
  bool update();
  
  // Kalman Filter Variables
  std::vector<double> state_;
  std::vector<double> cov_;
  double odom_innov_v_;
  double odom_innov_w_;
  double odom_cov_v_;
  double odom_cov_w_;
  double imu_innov_a_;
  double imu_innov_w_;
  double imu_cov_a_;
  double imu_cov_w_;
private:
  // Kalman filter functions
  bool predictKF(std::vector<double> *state_pre, std::vector<double> *cov_pre);
  bool measureKF(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                 std::vector<double> *state_post, std::vector<double> *cov_post);
  bool measureUpdateEncoder(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                           std::vector<double> *state_post, std::vector<double> *cov_post);
  bool measureUpdateIMU(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                       std::vector<double> *state_post, std::vector<double> *cov_post);
  
  // Kalman Filter parameters
  double odom_R_v_;
  double odom_R_w_;
  double imu_R_a_;
  double imu_R_w_;
  double proc_Q_v_;
  double proc_Q_w_;
  double proc_Q_a_;
  double proc_Q_wdot_;
  bool odom_init_;
  bool imu_init_;
  double dt_;
  
  
  // Update variables
  double odom_v_;
  double odom_w_;
  double imu_a_;
  double imu_w_;
  bool odom_new_;
  bool imu_new_;
};

KalmanVelocity::KalmanVelocity():
state_(4),cov_(16)
{
  odom_init_ = false;
  odom_new_ = false;
  imu_init_ = false;
  imu_new_ = false;
  odom_innov_v_ = 0;
  odom_innov_w_ = 0;
  imu_innov_a_ = 0;
  imu_innov_w_ = 0;
  odom_cov_v_ = 0;
  odom_cov_w_ = 0;
  imu_cov_a_ = 0;
  imu_cov_w_ = 0;
};

bool KalmanVelocity::initialize(double dt, double proc_var_v, double proc_var_w, double proc_var_a, double proc_var_wdot)
{
  dt_ = dt; 

  // Initialize covariance diagonal
  cov_[0]  = 1;
  cov_[5]  = 1;
  cov_[10] = 1;
  cov_[15] = 1;
  
  // Initialize Process Noise
  proc_Q_v_ = proc_var_v;
  proc_Q_w_ = proc_var_w;
  proc_Q_a_ = proc_var_a;
  proc_Q_wdot_ = proc_var_wdot;  
  return true;
}

bool KalmanVelocity::initializeEncoderNoise(double odom_var_v, double odom_var_w) 
{
  odom_R_v_ = odom_var_v;
  odom_R_w_ = odom_var_w;
  odom_init_ = true;
  return true;
}

bool KalmanVelocity::initializeIMUNoise(double imu_var_a, double imu_var_w)
{
  imu_R_a_ = imu_var_a;
  imu_R_w_ = imu_var_w;
  imu_init_ = true;
  return true;
}

bool KalmanVelocity::addMeasurementEncoders(double odom_v, double odom_w)
{
  if (!odom_init_)
  {
    ROS_ERROR("Tried to add Encoder measurement before initializing");
    return false;
  }
  odom_v_ = odom_v;
  odom_w_ = odom_w;
  odom_new_ = true;
  return true;
}

bool KalmanVelocity::addMeasurementIMU(double imu_a, double imu_w)
{
  if (!imu_init_)
  {
    ROS_ERROR("Tried to add IMU measurement before initializing");
    return false;
  }
  imu_a_ = imu_a;
  imu_w_ = imu_w;
  imu_new_ = true;
  return true;
}

bool KalmanVelocity::update()
{
  // Perform the prediction and measurement update states for the Kalman filter
  
  std::vector<double> state_pre(4,0);
  std::vector<double> state_post(4,0);
  std::vector<double> cov_pre(16,0);
  std::vector<double> cov_post(16,0);
  
  ROS_INFO("Updating Kalman Filter");
  //ROS_INFO("Initial State: [ %f, %f, %f, %f]", state_[0], state_[1], state_[2], state_[3]);
  //ROS_INFO("Initial Cov Diag: [ %f, %f, %f, %f]", cov_[0], cov_[5], cov_[10], cov_[15]);
  predictKF(&state_pre, &cov_pre);
  //ROS_INFO("State Pre:  [ %f, %f, %f, %f]", state_pre[0], state_pre[1], state_pre[2], state_pre[3]);
  //ROS_INFO("Cov Pre: [ %f, %f, %f, %f]", cov_pre[0], cov_pre[5], cov_pre[10], cov_pre[15]);
  
  if (odom_new_ || imu_new_)
  {
    // measureKF 
    measureKF(state_pre, cov_pre, &state_post, &cov_post);
  }
  else
  {
    // If no updates, save the predicted state and covariance
    state_post = state_pre;
    cov_post   = cov_pre;
  }
  
  state_ = state_post;
  cov_   = cov_post;
  
  //ROS_INFO("State After Update: [ %f, %f, %f, %f]", state_[0], state_[1], state_[2], state_[3]);
  //ROS_INFO("Initial Cov After Update: [ %f, %f, %f, %f]", cov_[0], cov_[5], cov_[10], cov_[15]);
  
  return true;
}


bool KalmanVelocity::predictKF(std::vector<double> *state_pre, std::vector<double> *cov_pre)
{
  //TODO: Verify that state_pre and cov_pre are valid
  
  // Propagate the state to X_pre
  state_pre->at(0) = state_[0] + state_[2]*dt_;  // v_new = v + a*dt
  state_pre->at(1) = state_[1] + state_[3]*dt_;  // w_new = w + w_dot*dt
  state_pre->at(2) = state_[2];                // a_new = a
  state_pre->at(3) = state_[3];                // w_dot_new = w_dot
  //ROS_INFO("dt: %f, Q_v: %f, Q_w: %f, Q_a: %f, Q_wdot: %f", dt_, proc_Q_v_, proc_Q_w_, proc_Q_a_, proc_Q_wdot_);
  
  // Propagate Covariance to P_pre (after state update): All elements
  cov_pre->at(0)  = cov_[0] + cov_[8]*dt_  + cov_[2]*dt_ + cov_[10]*dt_*dt_ + proc_Q_v_*dt_;
  cov_pre->at(2)  = cov_[2] + cov_[10]*dt_;
  cov_pre->at(5)  = cov_[5] + cov_[13]*dt_ + cov_[7]*dt_ + cov_[15]*dt_*dt_ + proc_Q_w_*dt_;
  cov_pre->at(7)  = cov_[7] + cov_[15]*dt_;
  cov_pre->at(8)  = cov_[10]*dt_;
  cov_pre->at(10) = cov_[10] + proc_Q_a_*dt_;
  cov_pre->at(13) = cov_[15]*dt_;
  cov_pre->at(15) = cov_[15] + proc_Q_wdot_*dt_;
  
  return true;
}

bool KalmanVelocity::measureKF(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                              std::vector<double> *state_post, std::vector<double> *cov_post)
{
  std::vector<double> state_in(4,0);
  std::vector<double> cov_in(16,0);
  
  state_in = state_pre;
  cov_in = cov_pre;
  *state_post = state_pre;
  *cov_post = cov_pre;
  
  if (odom_new_)
  {
    measureUpdateEncoder(state_in, cov_in, state_post, cov_post);
    odom_new_ = false;
  }
  
  state_in = *state_post;
  cov_in = *cov_post;
  
  if (imu_new_)
  {
    measureUpdateIMU(state_in, cov_in, state_post, cov_post);
    imu_new_ = false;
  }
  
  return true;
}

bool KalmanVelocity::measureUpdateEncoder(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                                          std::vector<double> *state_post, std::vector<double> *cov_post)
{
  // ENCODER UPDATE  
  // Propagate Covariance to P+ (after measurement update): All elements
  // Encoder update only
  odom_cov_v_ = cov_pre[0] + odom_R_v_;
  odom_cov_w_ = cov_pre[5] + odom_R_w_;
  cov_post->at(0)  = cov_pre[0] - cov_pre[0] * cov_pre[0] / odom_cov_v_; 
  cov_post->at(2)  = cov_pre[2] - cov_pre[0] * cov_pre[2] / odom_cov_v_; 
  cov_post->at(5)  = cov_pre[5] - cov_pre[5] * cov_pre[5] / odom_cov_w_; 
  cov_post->at(7)  = cov_pre[7] - cov_pre[5] * cov_pre[7] / odom_cov_w_; 
  cov_post->at(8)  = cov_pre[8] - cov_pre[8] * cov_pre[0] / odom_cov_v_; 
  cov_post->at(10) = cov_pre[10]- cov_pre[8] * cov_pre[2] / odom_cov_v_; 
  cov_post->at(13) = cov_pre[13]- cov_pre[13]* cov_pre[5] / odom_cov_w_; 
  cov_post->at(15) = cov_pre[15]- cov_pre[13]* cov_pre[7] / odom_cov_w_; 

  // Calculate odometry residual
  odom_innov_v_ = odom_v_ - state_pre[0];
  odom_innov_w_ = odom_w_ - state_pre[1];

  // Measurement update
  state_post->at(0) = state_pre[0] + odom_innov_v_ * cov_[0] / odom_R_v_;
  state_post->at(1) = state_pre[1] + odom_innov_w_ * cov_[5] / odom_R_w_;
  state_post->at(2) = state_pre[2] + odom_innov_v_ * cov_[8] / odom_R_v_;
  state_post->at(3) = state_pre[3] + odom_innov_w_ * cov_[13]/ odom_R_w_;  

  //ROS_INFO("Odom meas: V: %f, W: %f", odom_v_, odom_w_);
  return true;
}

bool KalmanVelocity::measureUpdateIMU(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                                       std::vector<double> *state_post, std::vector<double> *cov_post)
{  
  //NOTE: I commented out the updates related to the accelerometer. They werent working for some reason
  //     Add these back in the future. There may be sme more calibration needed. It looked like bad data,
  //     Or bad math/code. But the gyro and the encoders work, so I'm leaning towards bad data :) 
  
  // IMU UPDATE
  // Propagate Covariance to P+ (after measurement update): All elements
  // IMU update only
  imu_cov_a_ = cov_pre[10]+ imu_R_a_;
  imu_cov_w_ = cov_pre[5] + imu_R_w_;
  cov_post->at(5)  = cov_pre[5] - cov_pre[5] * cov_pre[5] / imu_cov_w_; 
  cov_post->at(7)  = cov_pre[7] - cov_pre[5] * cov_pre[7] / imu_cov_w_; 
  cov_post->at(13) = cov_pre[13]- cov_pre[13]* cov_pre[5] / imu_cov_w_; 
  cov_post->at(15) = cov_pre[15]- cov_pre[13]* cov_pre[7] / imu_cov_w_;
  //cov_post->at(0)  = cov_pre[0] - cov_pre[2] * cov_pre[8] / imu_cov_a_; 
  //cov_post->at(2)  = cov_pre[2] - cov_pre[2] * cov_pre[10]/ imu_cov_a_; 
  //cov_post->at(8)  = cov_pre[8] - cov_pre[10]* cov_pre[8] / imu_cov_a_; 
  //cov_post->at(10) = cov_pre[10]- cov_pre[10]* cov_pre[10]/ imu_cov_a_; 
      
  // Calculate imu residual
  imu_innov_a_ = imu_a_ - state_pre[3];
  imu_innov_w_ = imu_w_ - state_pre[1];
  
  // Measurement update
  state_post->at(1) = state_pre[1] + imu_innov_w_ * cov_[5] / imu_R_w_;
  state_post->at(3) = state_pre[3] + imu_innov_w_ * cov_[13]/ imu_R_w_;
  //state_post->at(0) = state_pre[0] + imu_innov_a_ * cov_[2] / imu_R_a_;
  //state_post->at(2) = state_pre[2] + imu_innov_a_ * cov_[10]/ imu_R_a_;
  
  //ROS_INFO("IMU meas: a: %f, W: %f", imu_a_, imu_w_);  
  return true;
}

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
  
  // TODO: Check that looprate exists
  bool rval = true;
  rval &= central_filter_.initialize(1/loop_rate_, proc_var_v_, proc_var_w_, proc_var_a_, proc_var_wdot_);
  rval &= central_filter_.initializeEncoderNoise(odom_var_v_, odom_var_w_);
  rval &= central_filter_.initializeIMUNoise(imu_var_a_, imu_var_w_);
  
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
      if (initialized_cnt_++ > 100)
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
      
      central_filter_.addMeasurementEncoders(odom_v, odom_w);
      central_filter_.addMeasurementIMU(x_accel, imu_w);
      central_filter_.update();
          
      
      test.corrected_imu_x = x_accel;
      test.corrected_imu_y = y_accel;
      test.kf_v = central_filter_.state_[0];
      test.kf_w = central_filter_.state_[1];
      test.kf_a = central_filter_.state_[2];
      test.kf_wdot = central_filter_.state_[3];
      test.innov_v = central_filter_.odom_innov_v_;
      test.innov_w = central_filter_.odom_innov_w_;
      test.innov_imu_a = central_filter_.imu_innov_a_;
      test.innov_imu_w = central_filter_.imu_innov_w_;
      test.innov_bound_v = 3*sqrt(central_filter_.odom_cov_v_); // bound on innovation is 3*sigma(v)
      test.innov_bound_w = 3*sqrt(central_filter_.odom_cov_w_); // bound on innovation is 3*sigma(w)
      test.innov_bound_imu_a = 3*sqrt(central_filter_.imu_cov_a_); // bound on innovation is 3*sigma(a)
      test.innov_bound_imu_w = 3*sqrt(central_filter_.imu_cov_w_); // bound on innovation is 3*sigma(w)
      
      double slip_enc_v, slip_enc_w, slip_gyro, slip_accel = 0;
      if (fabs(central_filter_.odom_cov_v_)>.001)
        slip_enc_v = fabs(central_filter_.odom_innov_v_ / sqrt(central_filter_.odom_cov_v_) );
      if (fabs(central_filter_.odom_cov_w_)>.001)
        slip_enc_w = fabs(central_filter_.odom_innov_w_ / sqrt(central_filter_.odom_cov_w_) );
      if (fabs(central_filter_.imu_cov_w_)>.001)
        slip_gyro  = fabs(central_filter_.imu_innov_w_ / sqrt(central_filter_.imu_cov_w_) );
      if (fabs(central_filter_.imu_cov_a_)>.001)
        slip_accel = fabs(central_filter_.imu_innov_a_ / sqrt(central_filter_.imu_cov_a_) );
      
      double max_innov = std::max(std::max(std::max(slip_enc_v,slip_enc_w),slip_gyro),slip_accel);
      slip.slip_enc_v = slip_enc_v;
      slip.slip_enc_w = slip_enc_v;
      slip.slip_gyro  = slip_gyro;
      slip.slip_accel = slip_accel;
      if (max_innov > 3)
        slip.slip = 1;
      else
        slip.slip = 0;
      
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
