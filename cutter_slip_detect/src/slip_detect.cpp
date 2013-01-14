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
 *   - gps_pose (geometry_msgs/PoseStamped): gps output in map frame
 *  
 * Publishes:
 *   - cwru/testing
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
  void getGPSLeverarm();

  // Ros Classes
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber gps_sub_;
  ros::Publisher test_pub_;
  ros::Publisher slip_pub_;
  ros::NodeHandle nh_;
  tf::TransformListener listener;
  
  RunningStat imu_stat_x_;
  RunningStat imu_stat_y_;

  // Kalman Filter parameters
  double odom_var_v_;
  double odom_var_w_;
  double imu_var_a_;
  double imu_var_w_;
  double gps_var_v_;
  double proc_var_v_;
  double proc_var_w_;
  double proc_var_a_;
  double proc_var_wdot_;
  double gps_leverarm_;
  
  KalmanVelocity central_filter_;
  KalmanVelocity kf_enc_;
  KalmanVelocity kf_aux_;

  double loop_rate_;
 
  // Store odom and imu values from callbacks
  nav_msgs::Odometry last_odom_;
  sensor_msgs::Imu   last_imu_;
  geometry_msgs::Point old_gps_;
  geometry_msgs::Point new_gps_;
  double prev_gps_vel_fix_;
  bool imu_new_;
  bool odom_new_;
  bool gps_new_;
  bool use_gps_;
  bool leverarm_valid_;
  
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
  
  void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& gps)
  {
    old_gps_ = new_gps_;
    new_gps_ = gps->pose.position;
    gps_new_ = true;
  };

};

SlipDetect::SlipDetect()
{
  loop_rate_ = 0;
  initialized_cnt_ = 0;
  initialized_ = false;
  odom_new_ = false;
  imu_new_  = false;
  gps_new_  = false;
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom",10,&SlipDetect::odomCallback,this);
  imu_sub_  = nh_.subscribe<sensor_msgs::Imu>("imu/data",10,&SlipDetect::imuCallback,this);
  gps_sub_  = nh_.subscribe<geometry_msgs::PoseStamped>("gps_pose",10,&SlipDetect::gpsCallback,this);
  test_pub_ = nh_.advertise<cutter_msgs::Testing>("cwru/testing",1);
  slip_pub_ = nh_.advertise<cutter_msgs::SlipStatus>("cwru/slip",1);
};

bool SlipDetect::lookupParams()
{
  
  bool rval = ros::param::get("~odom_var_v", odom_var_v_)
           && ros::param::get("~odom_var_w", odom_var_w_)
           && ros::param::get("~imu_var_a",  imu_var_a_)
           && ros::param::get("~imu_var_w",  imu_var_w_)
           && ros::param::get("~gps_var_v",  gps_var_v_)
           && ros::param::get("~use_gps", use_gps_)
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
    return false;
  }
  
  bool rval = true;
  rval &= central_filter_.initialize(1/loop_rate_, proc_var_v_, proc_var_w_, proc_var_a_, proc_var_wdot_);
  rval &= central_filter_.initializeEncoderNoise(odom_var_v_, odom_var_w_);
  rval &= central_filter_.initializeIMUNoise(imu_var_a_, imu_var_w_);
  
  rval &= kf_enc_.initialize(1/loop_rate_, proc_var_v_, proc_var_w_, proc_var_a_, proc_var_wdot_);
  rval &= kf_enc_.initializeEncoderNoise(odom_var_v_, odom_var_w_);
  
  rval &= kf_aux_.initialize(1/loop_rate_, proc_var_v_, proc_var_w_, proc_var_a_, proc_var_wdot_);
  rval &= kf_aux_.initializeIMUNoise(imu_var_a_, imu_var_w_);
  rval &= kf_aux_.initializeGPSNoise(gps_var_v_);
  
  return rval;
}

void SlipDetect::getGPSLeverarm()
{
  tf::StampedTransform transform;
  try{
    listener.waitForTransform("/base_link", "/base_gps", ros::Time(0), ros::Duration(10.0) );
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

void SlipDetect::filter()
{  
  double odom_v, odom_w, imu_x, imu_y, imu_w, gps_vel_fix, gps_vel;
  odom_v = last_odom_.twist.twist.linear.x;
  odom_w = last_odom_.twist.twist.angular.z;
  imu_x  = last_imu_.linear_acceleration.x;
  imu_y  = last_imu_.linear_acceleration.y;
  imu_w  = last_imu_.angular_velocity.z * M_PI / 180;
  gps_vel_fix = prev_gps_vel_fix_;
  
  double x_accel, y_accel;
  x_accel = imu_x - imu_stat_x_.Mean();
  y_accel = imu_y - imu_stat_y_.Mean();
  
  cutter_msgs::Testing test;
  cutter_msgs::SlipStatus slip;

/*
  // DONT WORRY ABOUT INITIALIZING CAUSE WE'RE NOT USING ACCELEROMETER
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
    
  } */
  
  bool no_update = true;
  if (odom_new_ && imu_new_)
  {
    no_update = false;
    
    //central_filter_.addMeasurementEncoders(odom_v, odom_w);
    //central_filter_.addMeasurementIMU(x_accel, imu_w);
    //central_filter_.update();
    
    // Update the Encoder Filter
    kf_enc_.addMeasurementEncoders(odom_v, odom_w);
    kf_enc_.update();
    
    // Update the Auxillary Filter
    kf_aux_.addMeasurementIMU(x_accel, imu_w); // Note x_accel not used in the kalman filter
    if (gps_new_ && use_gps_)
    {
      if (!leverarm_valid_)
      {
        getGPSLeverarm();     
      }
      else
      {
        double xoff, yoff, gps_dist;
        xoff = new_gps_.x - old_gps_.x;
        yoff = new_gps_.y - old_gps_.y;
        // GpsDist = new gps - old gps
        gps_dist = sqrt( xoff*xoff + yoff*yoff );
        // GpsVel  = GpsDist / 0.1   <--- Distance / GPS DT where GPS DT = 10 Hz or 0.1 s
        gps_vel  = gps_dist / 0.1;
        // GpsVelFix = sqrt( GpsVel^2 - (r*w)^2 )   <--- Ie, subtract out component of motion due to angular velocity
        gps_vel_fix = sqrt( gps_vel*gps_vel  -  gps_leverarm_*gps_leverarm_*imu_w*imu_w );
        if (odom_v < 0)
          gps_vel_fix = - gps_vel_fix;
        ROS_INFO("GPS Velocity: %f, Odom Velocity: %f", gps_vel_fix, odom_v);
        prev_gps_vel_fix_ = gps_vel_fix;
        kf_aux_.addMeasurementGPS(gps_vel_fix);
        gps_new_ = false; 
      }
    }
    kf_aux_.update();
    
    // Fuse the two Filters
    double Ptot_v, Ptot_w, Ptot_a, Ptot_wdot;
    double Xtot_v, Xtot_w, Xtot_a, Xtot_wdot;
    Ptot_v    = kf_enc_.cov_[0]  * kf_aux_.cov_[0]  / (kf_enc_.cov_[0]  + kf_aux_.cov_[0]);
    Ptot_w    = kf_enc_.cov_[5]  * kf_aux_.cov_[5]  / (kf_enc_.cov_[5]  + kf_aux_.cov_[5]);
    Ptot_a    = kf_enc_.cov_[10] * kf_aux_.cov_[10] / (kf_enc_.cov_[10] + kf_aux_.cov_[10]);
    Ptot_wdot = kf_enc_.cov_[15] * kf_aux_.cov_[15] / (kf_enc_.cov_[15] + kf_aux_.cov_[15]);
    
    Xtot_v    = Ptot_v / kf_enc_.cov_[0]  * kf_enc_.state_[0]  +  Ptot_v / kf_aux_.cov_[0]  * kf_aux_.state_[0];
    Xtot_w    = Ptot_w / kf_enc_.cov_[5]  * kf_enc_.state_[1]  +  Ptot_w / kf_aux_.cov_[5]  * kf_aux_.state_[1];
    Xtot_a    = Ptot_a / kf_enc_.cov_[10] * kf_enc_.state_[2]  +  Ptot_a / kf_aux_.cov_[10] * kf_aux_.state_[2];
    Xtot_wdot = Ptot_wdot / kf_enc_.cov_[15] * kf_enc_.state_[3]  +  Ptot_wdot / kf_aux_.cov_[15] * kf_aux_.state_[3];
    
    ROS_INFO("Enc Cov Diag: [ %f, %f, %f, %f]", kf_enc_.cov_[0], kf_enc_.cov_[5], kf_enc_.cov_[10], kf_enc_.cov_[15]);
    ROS_INFO("Enc State: [ %f, %f, %f, %f]", kf_enc_.state_[0], kf_enc_.state_[1], kf_enc_.state_[2], kf_enc_.state_[3]);
    ROS_INFO("Aux Cov Diag: [ %f, %f, %f, %f]", kf_aux_.cov_[0], kf_aux_.cov_[5], kf_aux_.cov_[10], kf_aux_.cov_[15]);
    ROS_INFO("Aux State: [ %f, %f, %f, %f]", kf_aux_.state_[0], kf_aux_.state_[1], kf_aux_.state_[2], kf_aux_.state_[3]);
    ROS_INFO("Tot Cov Diag: [ %f, %f, %f, %f]", Ptot_v, Ptot_w, Ptot_a, Ptot_wdot);
    ROS_INFO("Tot State: [ %f, %f, %f, %f]", Xtot_v, Xtot_w, Xtot_a, Xtot_wdot);
    //ROS_INFO("P_v_enc: %f, P_v_aux: %f, P_a_enc: %f, P_a_aux: %f", Ptot_v / kf_enc_.cov_[0], Ptot_v / kf_aux_.cov_[0], Ptot_a / kf_enc_.cov_[10], Ptot_a / kf_aux_.cov_[10]);
    
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
    //test.innov_v = kf_aux_.state_[0] - odom_v;
    //test.innov_w = kf_aux_.state_[1] - odom_w;
    //test.innov_imu_a = kf_enc_.state_[2] - x_accel;
    //test.innov_imu_w = kf_enc_.state_[1] - imu_w;    
    test.innov_bound_v = 3*(sqrt(Ptot_v) + odom_var_v_); 
    test.innov_bound_w = 3*(sqrt(Ptot_w) + odom_var_w_); 
    test.innov_bound_imu_a = 3*(sqrt(Ptot_a) + imu_var_a_); 
    test.innov_bound_imu_w = 3*(sqrt(Ptot_w) + imu_var_w_);
    test.gpsvelfix = gps_vel_fix;
    test.gpsvel = gps_vel;
    test.odomvel = odom_v;
     
    // Calculate the certainty for each measurement:
    //     certainty = measurement / sqrt(covariance)
    // if meas > 3*sqrt(cov), then there's like a 1% chance or less that 
    // the measurement is correct
    double slip_enc_v, slip_enc_w, slip_gyro, slip_accel, slip_gps_v = 0;
    if ( (sqrt(Ptot_v) + odom_var_v_) > .001)
      slip_enc_v = fabs(test.innov_v) / (sqrt(Ptot_v) + odom_var_v_);
    if ( (sqrt(Ptot_w) + odom_var_w_) > .001)
      slip_enc_w = fabs(test.innov_w) / (sqrt(Ptot_w) + odom_var_w_);
    if ( (sqrt(Ptot_w) + imu_var_w_) > .001)
      slip_gyro  = fabs(test.innov_imu_w) / (sqrt(Ptot_w) + imu_var_w_) ;
    if ( (sqrt(Ptot_a) + imu_var_a_) > .001)
      slip_accel = fabs(test.innov_imu_a) / (sqrt(Ptot_a) + imu_var_a_) ;
    if ( (sqrt(Ptot_v) + gps_var_v_) > .001)
      slip_gps_v = fabs(Xtot_v - gps_vel_fix) / (sqrt(Ptot_v) + gps_var_v_) ;
    
    double max_innov = std::max(std::max(std::max(std::max(slip_enc_v, slip_gps_v),slip_enc_w),slip_gyro),slip_accel);
    test.slip_enc_v = slip_enc_v/3;
    test.slip_enc_w = slip_enc_w/3;
    test.slip_gyro  = slip_gyro/3;
    test.slip_accel = slip_accel/3;
    test.slip_gps_v = slip_gps_v/3;
    test.slip_max = max_innov/3;
    slip.slip_max = max_innov/3;
    if (max_innov > 3) // Slip if the max innovation is > 3
      slip.slip = 1;
    else
      slip.slip = 0;
      
    // TODO: If we slip, maybe take the other filter out of the fused measurement??
    
    // TODO: Implement an "integral term" on the slip.. If we're consistently have a larger
    //       than expected innovation, this could indicate a slip
    
    // TODO: Have this node publish the filtered odometry?? 
    
    slip_pub_.publish(slip);
    
    imu_new_ = false;
    odom_new_ = false;
    gps_new_ = false;
  }

  if (no_update)
  {
    // Populate the kalman filter variables with the last state
    test.corrected_imu_x = x_accel;
    test.corrected_imu_y = y_accel;
    test.kf_v = 0;
    test.kf_w = 0;
    test.kf_a = 0;
    test.kf_wdot = 0;
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
