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
  void setLoopRate(double rate) { loop_rate_ = rate; };

private:
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher test_pub_;
  ros::NodeHandle nh_;
  RunningStat imu_stat_x_;
  RunningStat imu_stat_y_;

  nav_msgs::Odometry last_odom_;
  sensor_msgs::Imu   last_imu_;

  double loop_rate_;
  
  double odom_v_;
  double odom_w_;
  double imu_w_;
  double imu_x_;
  double imu_y_;
  
  bool imu_new_;
  bool odom_new_;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
  {
    odom_v_ = odom->twist.twist.linear.x;
    odom_w_ = odom->twist.twist.angular.z;
    odom_new_ = true;
  };

  void imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
  {
    imu_w_ = imu->angular_velocity.z;
    imu_x_ = imu->linear_acceleration.x;
    imu_y_ = imu->linear_acceleration.y; 
    imu_new_ = true;
  };

};

SlipDetect::SlipDetect()
{
  odom_new_ = false;
  imu_new_  = false;
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom",10,&SlipDetect::odomCallback,this);
  imu_sub_  = nh_.subscribe<sensor_msgs::Imu>("imu/data",10,&SlipDetect::imuCallback,this);
  test_pub_ = nh_.advertise<cutter_msgs::Testing>("cwru/testing",1);
};

void SlipDetect::filter()
{
  if (imu_new_)
  {
    if (abs(odom_v_) < .001 && abs(odom_w_ < .001))
    {
      // Filter the IMU when the wheels report no motion (assume the robot is not moving) 
      imu_stat_x_.Push(imu_x_);
      imu_stat_y_.Push(imu_y_);
      ROS_INFO("Imu X: %f, X average: %f, Stddev: %f, X-avg: %f", 
                    imu_x_,  imu_stat_x_.Mean(), imu_stat_x_.StandardDeviation(), imu_x_-imu_stat_x_.Mean());
      ROS_INFO("Imu Y: %f, Y average: %f, Stddev: %f, Y-avg: %f", 
                    imu_y_,  imu_stat_y_.Mean(), imu_stat_y_.StandardDeviation(), imu_y_-imu_stat_y_.Mean());
    }
    imu_new_ = false;
  }
  
  double x_accel, y_accel;
  
  x_accel = imu_x_ - imu_stat_x_.Mean();
  y_accel = imu_y_ - imu_stat_y_.Mean();
  
  cutter_msgs::Testing test;
  test.corrected_imu_x = x_accel;
  test.corrected_imu_y = y_accel;
  test_pub_.publish(test);
  
  // When the robot is moving, implement the kalman filter thing, evaluate the feasibility..
  
};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "slip_detect"); //Init ROS
  SlipDetect detector; //Construct class

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
