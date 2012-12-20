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
#include <vector>

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
  ros::NodeHandle nh_;
  
  RunningStat imu_stat_x_;
  RunningStat imu_stat_y_;


  double loop_rate_;
 
  // Store odom and imu values from callbacks
  nav_msgs::Odometry last_odom_;
  sensor_msgs::Imu   last_imu_;
  bool imu_new_;
  bool odom_new_;
  
  // IMU Initialization
  bool initialized_;
  int initialized_cnt_;
  
  // Kalman Filter parameters
  double odom_var_v_;
  double odom_var_w_;
  double imu_var_a_;
  double imu_var_w_;
  double proc_var_v_;
  double proc_var_w_;
  double proc_var_a_;
  double proc_var_wdot_;
  
  // Kalman Filter Variables
  std::vector<double> state_;
  std::vector<double> cov_;
  

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

SlipDetect::SlipDetect():
  state_(4,0), cov_(16,0)
{
  initialized_cnt_ = 0;
  initialized_ = false;
  odom_new_ = false;
  imu_new_  = false;
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom",10,&SlipDetect::odomCallback,this);
  imu_sub_  = nh_.subscribe<sensor_msgs::Imu>("imu/data",10,&SlipDetect::imuCallback,this);
  test_pub_ = nh_.advertise<cutter_msgs::Testing>("cwru/testing",1);
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
  cov_[0]  = 1;
  cov_[5]  = 1;
  cov_[10] = 1;
  cov_[15] = 1;
  
  return true;
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
  
  double odom_cov_v, odom_cov_w, odom_v_err, odom_w_err = 0;
  double imu_cov_a,  imu_cov_w,  imu_a_err,  imu_w_err = 0;
  
  cutter_msgs::Testing test;

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
   /* if (fabs(odom_v) < .001 && fabs(odom_w) < .001)
    {
      // Currently do nothing with odometry if we're not moving??
      ROS_INFO("odom: No motion");
    }
    else
    {*/
    if (initialized_)
    {
      no_update = false;
      // When the robot is moving, implement the kalman filter thing..
      //double state_pre[4], state_post[4], cov_pre[16], cov_post[16];
      std::vector<double> state_pre(4,0);
      std::vector<double> cov_pre(16,0);
      double odom_std_v, odom_std_w;
      double dt = 1/loop_rate_;
      
      //ROS_INFO("Initial State: [ %f, %f, %f, %f]", state_[0], state_[1], state_[2], state_[3]);
      // Propagate the state to X_pre
      state_pre[0] = state_[0] + state_[2]*dt;  // v_new = v + a*dt
      state_pre[1] = state_[1] + state_[3]*dt;  // w_new = w + w_dot*dt
      state_pre[2] = state_[2];                // a_new = a
      state_pre[3] = state_[3];                // w_dot_new = w_dot
      //ROS_INFO("State Pre:  [ %f, %f, %f, %f]", state_pre[0], state_pre[1], state_pre[2], state_pre[3]);
      
      // Propagate Covariance: Diagonal elements only
      /*cov_pre[0]  = cov_[0] + cov_[10]*dt*dt;
      cov_pre[5]  = cov_[5] + cov_[15]*dt*dt;
      cov_pre[10] = cov_[10];
      cov_pre[15] = cov_[15];*/
      
      // Propagate Covariance to P_pre (after state update): All elements
      //ROS_INFO("Initial Cov Diag: [ %f, %f, %f, %f]", cov_[0], cov_[5], cov_[10], cov_[15]);
      cov_pre[0]  = cov_[0] + cov_[8]*dt  + cov_[2]*dt + cov_[10]*dt*dt + proc_var_v_*dt;
      cov_pre[2]  = cov_[2] + cov_[10]*dt;
      cov_pre[5]  = cov_[5] + cov_[13]*dt + cov_[7]*dt + cov_[15]*dt*dt + proc_var_w_*dt;
      cov_pre[7]  = cov_[7] + cov_[15]*dt;
      cov_pre[8]  = cov_[10]*dt;
      cov_pre[10] = cov_[10] + proc_var_a_*dt;
      cov_pre[13] = cov_[15]*dt;
      cov_pre[15] = cov_[15] + proc_var_wdot_*dt;
      //ROS_INFO("Cov Diag Pre: [ %f, %f, %f, %f]", cov_pre[0], cov_pre[5], cov_pre[10], cov_pre[15]);
      
      // ENCODER UPDATE      
      // Propagate Covariance to P+ (after measurement update): All elements
      // Encoder update only
      odom_cov_v = cov_pre[0] + odom_var_v_;
      odom_cov_w = cov_pre[5] + odom_var_w_;
      cov_[0]  = cov_pre[0] - cov_pre[0] * cov_pre[0] / odom_cov_v; 
      cov_[2]  = cov_pre[2] - cov_pre[0] * cov_pre[2] / odom_cov_v; 
      cov_[5]  = cov_pre[5] - cov_pre[5] * cov_pre[5] / odom_cov_w; 
      cov_[7]  = cov_pre[7] - cov_pre[5] * cov_pre[7] / odom_cov_w; 
      cov_[8]  = cov_pre[8] - cov_pre[8] * cov_pre[0] / odom_cov_v; 
      cov_[10] = cov_pre[10]- cov_pre[8] * cov_pre[2] / odom_cov_v; 
      cov_[13] = cov_pre[13]- cov_pre[13]* cov_pre[5] / odom_cov_w; 
      cov_[15] = cov_pre[15]- cov_pre[13]* cov_pre[7] / odom_cov_w; 
      
      // Calculate odometry residual
      odom_v_err = odom_v - state_pre[0];
      odom_w_err = odom_w - state_pre[1];
      
      // Measurement update
      state_[0] = state_pre[0] + odom_v_err * cov_[0] / odom_var_v_;
      state_[1] = state_pre[1] + odom_w_err * cov_[5] / odom_var_w_;
      state_[2] = state_pre[2] + odom_v_err * cov_[8] / odom_var_v_;
      state_[3] = state_pre[3] + odom_w_err * cov_[13]/ odom_var_w_;    
      
      //ROS_INFO("Cov_v: %f, Cov_w: %f, Odom_err_v: %f, odom_err_w: %f", cov_v, cov_w, odom_v_err, odom_w_err);
      //ROS_INFO("odom_var_v: %f, odom_var_w: %f", odom_var_v_, odom_var_w_);
      //ROS_INFO("Cov For Meas Update: [ %f, %f, %f, %f]", cov_[0], cov_[5], cov_[8], cov_[13]);
      ROS_INFO("Odom State Post: [ %f, %f, %f, %f]", state_[0], state_[1], state_[3], state_[4]);
      ROS_INFO("Odom Cov Diag Post: [ %f, %f, %f, %f]", cov_[0], cov_[5], cov_[10], cov_[15]);
      ROS_INFO("Odom meas: V: %f, W: %f", odom_v, odom_w);
      
      // Reassign state_pre to state
      state_pre = state_;
      cov_pre   = cov_;
      
      // IMU UPDATE
      // Propagate Covariance to P+ (after measurement update): All elements
      // IMU update only
      imu_cov_a = cov_pre[10]+ imu_var_a_;
      imu_cov_w = cov_pre[5] + imu_var_w_;
      //cov_[0]  = cov_pre[0] - cov_pre[2] * cov_pre[8] / imu_cov_a; 
      //cov_[2]  = cov_pre[2] - cov_pre[2] * cov_pre[10]/ imu_cov_a; 
      cov_[5]  = cov_pre[5] - cov_pre[5] * cov_pre[5] / imu_cov_w; 
      cov_[7]  = cov_pre[7] - cov_pre[5] * cov_pre[7] / imu_cov_w; 
      //cov_[8]  = cov_pre[8] - cov_pre[10]* cov_pre[8] / imu_cov_a; 
      //cov_[10] = cov_pre[10]- cov_pre[10]* cov_pre[10] / imu_cov_a; 
      cov_[13] = cov_pre[13]- cov_pre[13]* cov_pre[5] / imu_cov_w; 
      cov_[15] = cov_pre[15]- cov_pre[13]* cov_pre[7] / imu_cov_w;
      
      // Calculate odometry residual
      imu_a_err = x_accel - state_pre[3];
      imu_w_err = imu_w - state_pre[1];
      
      // Measurement update
      //state_[0] = state_pre[0] + imu_a_err * cov_[2] / imu_var_a_;
      state_[1] = state_pre[1] + imu_w_err * cov_[5] / imu_var_w_;
      //state_[2] = state_pre[2] + imu_a_err * cov_[10]/ imu_var_a_;
      state_[3] = state_pre[3] + imu_w_err * cov_[13]/ imu_var_w_;
      
      ROS_INFO("IMU State Post: [ %f, %f, %f, %f]", state_[0], state_[1], state_[3], state_[4]);
      ROS_INFO("IMU Cov Diag Post: [ %f, %f, %f, %f]", cov_[0], cov_[5], cov_[10], cov_[15]);
      ROS_INFO("IMU meas: a: %f, W: %f", x_accel, imu_w);  
      ROS_INFO("IMU innov: a: %f, W: %f", imu_a_err, imu_w_err);  
      
      //state_ = state_post;
      //cov_   = cov_post;
          
      test.corrected_imu_x = x_accel;
      test.corrected_imu_y = y_accel;
      test.kf_v = state_[0];
      test.kf_w = state_[1];
      test.kf_a = state_[2];
      test.kf_wdot = state_[3];
      test.innov_v = odom_v_err;
      test.innov_w = odom_w_err;
      test.innov_imu_a = imu_a_err;
      test.innov_imu_w = imu_w_err;
      test.innov_bound_v = 3*sqrt(odom_cov_v); // bound on innovation is 3*sigma(v)
      test.innov_bound_w = 3*sqrt(odom_cov_w); // bound on innovation is 3*sigma(w)
      test.innov_bound_imu_a = 3*sqrt(imu_cov_a); // bound on innovation is 3*sigma(a)
      test.innov_bound_imu_w = 3*sqrt(imu_cov_w); // bound on innovation is 3*sigma(w)
      
    }
  }
  
  imu_new_ = false;
  odom_new_ = false;

  if (no_update)
  {
    // Populate the kalman filter variables with the last state
    test.corrected_imu_x = x_accel;
    test.corrected_imu_y = y_accel;
    test.kf_v = state_[0];
    test.kf_w = state_[1];
    test.kf_a = state_[2];
    test.kf_wdot = state_[3];
  }


  test_pub_.publish(test);
  
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "slip_detect"); //Init ROS
  SlipDetect detector; //Construct class

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

//  ros::spin(); // Run until interupted 
  double looprate = 20;
  ros::Rate loop_rate(looprate);
  detector.setLoopRate(looprate);
  while (ros::ok())
  {
    ros::spinOnce();
    detector.filter();
    loop_rate.sleep();
  }
};
