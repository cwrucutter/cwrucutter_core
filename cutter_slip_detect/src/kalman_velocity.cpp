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
 * kalman_velocity.cpp
 *   Provides a basic Kalman filter for Velocity for the mobile robot.
 *   Updates are available for odometry and IMU curretly
 * 
 *
 ********************************************************************************/

#include "math.h"
#include <vector>
#include <algorithm>
#include "kalman_velocity.h"

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
  
  // TODO: Make the predict and measure updates overrideable for virtual classes
  //       ... OR, take a look at open source KF libraries! BFL, etc. 
  //        It would probably be better organized that what I've got here
  
  // PREDICTION UPDATE
  ROS_INFO("Updating Kalman Filter");
  //ROS_INFO("Initial State: [ %f, %f, %f, %f]", state_[0], state_[1], state_[2], state_[3]);
  //ROS_INFO("Initial Cov Diag: [ %f, %f, %f, %f]", cov_[0], cov_[5], cov_[10], cov_[15]);
  predictKF(&state_pre, &cov_pre);
  //ROS_INFO("State Pre:  [ %f, %f, %f, %f]", state_pre[0], state_pre[1], state_pre[2], state_pre[3]);
  //ROS_INFO("Cov Pre: [ %f, %f, %f, %f]", cov_pre[0], cov_pre[5], cov_pre[10], cov_pre[15]);
  
  // MEASUREMENT UPDATE
  if (odom_new_ || imu_new_) // Only if new data is received
  {
    // measureKF 
    measureKF(state_pre, cov_pre, &state_post, &cov_post);
  }
  else
  {
    // Otherwise, use state_pre and cov_pre
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
  if (!(state_pre->size() && cov_pre->size()))
  {
    // I'm just assuming that if the vectors are initialized then they'll be the right size
    ROS_ERROR("Must initialize vectors in KalmanVelocity::predictKF()");
    return false;
  }

  // Propagate the state to X_pre
  state_pre->at(0) = state_[0] + state_[2]*dt_;  // v_new = v + a*dt
  state_pre->at(1) = state_[1] + state_[3]*dt_;  // w_new = w + w_dot*dt
  state_pre->at(2) = state_[2];                  // a_new = a
  state_pre->at(3) = state_[3];                  // w_dot_new = w_dot
  
  // Propagate Covariance to P_pre (after state update): All elements
  cov_pre->at(0)  = cov_[0] + cov_[8]*dt_  + cov_[2]*dt_ + cov_[10]*dt_*dt_ + proc_Q_v_*dt_;
  cov_pre->at(2)  = cov_[2] + cov_[10]*dt_;
  cov_pre->at(5)  = cov_[5] + cov_[13]*dt_ + cov_[7]*dt_ + cov_[15]*dt_*dt_ + proc_Q_w_*dt_;
  cov_pre->at(7)  = cov_[7] + cov_[15]*dt_;
  cov_pre->at(8)  = cov_[10]*dt_;
  cov_pre->at(10) = cov_[10] + proc_Q_a_*dt_;
  cov_pre->at(13) = cov_[15]*dt_;
  cov_pre->at(15) = cov_[15] + proc_Q_wdot_*dt_;
  
  limitCovariance(cov_pre, 15);
  
  return true;
}

bool KalmanVelocity::measureKF(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                              std::vector<double> *state_post, std::vector<double> *cov_post)
{
  // Measurement Update results are returned
  //  inside the populated state_post and cov_post
  
  if (!(state_pre->size() && cov_pre->size() && state_post.size() && cov_post.size()))
  {
    // I'm just assuming that if the vectors are initialized then they'll be the right size
    ROS_ERROR("Must initialize vectors in KalmanVelocity::measureKF()");
    return false;
  }
  
  std::vector<double> state_in(4,0);
  std::vector<double> cov_in(16,0);
  
  // Save vectors
  state_in = state_pre;
  cov_in = cov_pre;
  *state_post = state_pre;
  *cov_post = cov_pre;
  
  if (odom_new_) // Perform odometry update if necessary
  {
    measureUpdateEncoder(state_in, cov_in, state_post, cov_post);
    odom_new_ = false;
  }
  
  // Save vectors
  state_in = *state_post;
  cov_in = *cov_post;
  
  if (imu_new_) // Perform Auxillary update if necessary
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
  //     Or bad math/code. But the gyro and the encoders work, so I'm leaning towards bad data/calibration :) 
  
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

void KalmanVelocity::limitCovariance(std::vector<double> *vec, double limit)
{
  // vec should be size 16
  // Just limit the vector to the max uncertainty... Shouldnt change too much if the max uncertainty is reasonably large
  vec->at(0)  = std::min(vec->at(0) , limit);
  vec->at(2)  = std::min(vec->at(2) , limit);
  vec->at(5)  = std::min(vec->at(5) , limit);
  vec->at(7)  = std::min(vec->at(7) , limit);
  vec->at(8)  = std::min(vec->at(8) , limit);
  vec->at(10) = std::min(vec->at(10), limit);
  vec->at(13) = std::min(vec->at(13), limit);
  vec->at(15) = std::min(vec->at(15), limit);
}
