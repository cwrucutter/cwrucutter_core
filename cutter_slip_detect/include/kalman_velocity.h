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
 * kalman_velocity.h
 *   Provides a basic Kalman filter for Velocity for the mobile robot.
 *   Updates are available for odometry and IMU curretly
 * 
 *
 ********************************************************************************/

#include "ros/ros.h"
#include "math.h"
#include <vector>
#include <algorithm>


class KalmanVelocity
{
// Class for the Kalman Filter on velocity and accelerations, 
// Provides update methods for encoders and IMUs
public: 
  KalmanVelocity(); 
  bool initialize(double dt, double proc_var_v, double proc_var_w, double proc_var_a, double proc_var_wdot);
  bool initializeEncoderNoise(double odom_var_v, double odom_var_w);
  bool initializeIMUNoise(double imu_var_a, double imu_var_w); 
  bool initializeGPSNoise(double gps_var_v); 
  bool addMeasurementEncoders(double odom_v, double odom_w);
  bool addMeasurementIMU(double imu_a, double imu_w);
  bool addMeasurementGPS(double gps_v);
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
  double gps_innov_v_;
  double gps_cov_v_;
  
private:
  // Kalman filter functions
  bool predictKF(std::vector<double> *state_pre, std::vector<double> *cov_pre);
  bool measureKF(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                 std::vector<double> *state_post, std::vector<double> *cov_post);
  bool measureUpdateEncoder(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                           std::vector<double> *state_post, std::vector<double> *cov_post);
  bool measureUpdateIMU(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                       std::vector<double> *state_post, std::vector<double> *cov_post);
  bool measureUpdateGPS(const std::vector<double> &state_pre, const std::vector<double> &cov_pre,
                       std::vector<double> *state_post, std::vector<double> *cov_post);
                       
  void limitCovariance(std::vector<double> *vec, double limit);
  
  // Kalman Filter parameters
  double odom_R_v_;
  double odom_R_w_;
  double imu_R_a_;
  double imu_R_w_;
  double gps_R_v_;
  double proc_Q_v_;
  double proc_Q_w_;
  double proc_Q_a_;
  double proc_Q_wdot_;
  bool odom_init_;
  bool imu_init_;
  bool gps_init_;
  double dt_;
  
  
  // Update variables
  double odom_v_;
  double odom_w_;
  double imu_a_;
  double imu_w_;
  double gps_v_;
  bool odom_new_;
  bool imu_new_;
  bool gps_new_;
};
