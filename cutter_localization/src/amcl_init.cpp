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
 * amcl_initialize.cpp
 *   Published one initialpose for amcl to initialize
 * 
 * Subscribes:
 *   - gps_pose (geometry_msgs/PoseStamped): Current differential GPS reading
 *  
 * Publishes:
 *   - initialpose (geometry_msgs/PoseWithCovarianceStamped): Current Error stats for the localization solution
 *
 ********************************************************************************/

#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"

class GpsInitialize
{
public:
  GpsInitialize()
  {
    init_ = true;
    dgps_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("gps_pose",1,&GpsInitialize::msgCallback,this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);
  } ;
  bool initialized() { return init_; };

private:
  ros::Subscriber dgps_sub_;
  ros::Publisher pose_pub_;
  ros::NodeHandle nh_;
  bool init_;

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& pose) 
  {
    ROS_INFO("Message Received\n");
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    
    // Populate the initialization msg
    init_pose.header = pose->header;
    init_pose.pose.pose = pose->pose;
    init_pose.pose.covariance[0] = .25;
    init_pose.pose.covariance[7] = .25;
    init_pose.pose.covariance[35] = .069;
    
    // Publish the message
    if (init_)
    {
      pose_pub_.publish(init_pose);
      ROS_INFO("Publishing initialpose\n");
      init_ = false;
    }
    
  };

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "amcl_int"); //Init ROS
  ROS_INFO("Initializing using gps_pose");
  GpsInitialize init; 
  while (init.initialized() && ros::ok())
  {
    ros::spinOnce();
  }
  ROS_INFO("Sent initialpose. Exiting");
  return 0;
};
