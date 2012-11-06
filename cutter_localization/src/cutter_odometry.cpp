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

class CutterOdometry
{
  public:
    CutterOdometry();
    bool lookupParams();
    bool sendOdometry();
  
  private:
    void encCountCallback(const cutter_msgs::EncMsg::ConstPtr& enc);    
   
    ros::NodeHandle node_;
    
    ros::Publisher  odom_pub_;
    ros::Subscriber enc_sub_;
    
    double x_;
    double y_;
    double tht_;
  
    int enc_left_;
    int enc_right_;

    double track_;
    int ticks_per_m_left_;
    int ticks_per_m_right_;

};

CutterOdometry::CutterOdometry():
  track_(0.67), ticks_per_m_left_(20000), ticks_per_m_right_(20000)
{
  enc_left_  = 0;
  enc_right_ = 0;

  odom_pub_ = node_.advertise<nav_msgs::Odometry>("odom",1);
  enc_sub_  = node_.subscribe<cutter_msgs::EncMsg>("cwru/enc_count",100,&CutterOdometry::encCountCallback,this);
}

bool CutterOdometry::sendOdometry()
{
  ROS_INFO("Sending Odometry: %i, %i, %f, %i, %i",enc_left_,enc_right_,track_,ticks_per_m_right_,ticks_per_m_left_);
  //TODO Parse and send odometry here
  return true;
}

bool CutterOdometry::lookupParams()
{
  return ros::param::get("~track", track_)
      && ros::param::get("~tpm_left",ticks_per_m_left_)
      && ros::param::get("~tpm_right",ticks_per_m_right_);
}

void CutterOdometry::encCountCallback(const cutter_msgs::EncMsg::ConstPtr& enc)
{
  double start = ros::Time::now().toSec();

  if (!lookupParams())
  {
    ROS_WARN("Encoder parameters do not exist. Cannot parse encoders");
    return;
  }

  enc_right_ = enc->right; 
  enc_left_  = enc->left;
  double dT  = 50.0;
  double Dr, Dl, Vr, Vl;
  Dr = enc_right_ / ticks_per_m_right_;
  Dl = enc_left_  / ticks_per_m_left_;
  Vr = Dr / dT;
  Vl = Dl / dT; 

  double diff = Vr - Vl;
  double sum  = Vr + Vl;
  double xnew, ynew, thtn;
  if (diff < .000001)
  {
    xnew = x_ + sum/2 * cos(tht_)*dT;
    ynew = y_ + sum/2 * sin(tht_)*dT;
    thtn = tht_;
  }
  else
  {

    xnew = x_ + track_ * sum/(2*diff) * (sin(diff*dT/track_+tht_) - sin(tht_));
    ynew = y_ - track_ * sum/(2*diff) * (cos(diff*dT/track_+tht_) - cos(tht_));
    thtn = tht_ + diff/track_*dT;
  }
  
  double end = ros::Time::now().toSec();
    
  ROS_INFO("Encoders: R: %f L: %f x_: %f y_: %f tht_: %f",Dr,Dl,x_,y_,tht_);
  ROS_INFO("          xnew: %f ynew: %f thtn: %f duration: %f",xnew,ynew,thtn,end-start);
  x_ = xnew;
  y_ = ynew;
  tht_ = thtn;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry");
  CutterOdometry odometry;
  
  ros::spin();

  /*ros::Rate loop_rate(10.0);
  while (ros::ok())
  {
    if (odometry.lookupParams())
    {
      ros::spinOnce();
      odometry.sendOdometry();
    }
    else
    {
      ROS_WARN("Parameters not found");
    }
    loop_rate.sleep();
  }*/

  return 0;
}
