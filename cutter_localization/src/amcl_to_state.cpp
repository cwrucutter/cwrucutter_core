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
 * amcl_to_state.cpp
 *   Converts the amcl tf messages to our cutter_msgs/State, publishes at 10 Hz
 * 
 * Subscribes:
 *   - odom (nav_msgs/Odometry): Current odometry, used for velocity measurements
 *  
 * Publishes:
 *   - cwru/state (cutter_msgs/State): Current state
 *
 ********************************************************************************/

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "cutter_msgs/State.h"
#include "nav_msgs/Odometry.h"

class AmclToState
{
public:
  AmclToState() : tf_(),  target_frame_("map")
  {
    odom_sub_.subscribe(nh_, "odom", 10);
    tf_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&AmclToState::msgCallback, this, _1) );
    state_pub_ = nh_.advertise<cutter_msgs::State>("cwru/state",1);
  } ;

private:
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<nav_msgs::Odometry> * tf_filter_;
  ros::Publisher state_pub_;
  ros::NodeHandle nh_;
  std::string target_frame_;

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const nav_msgs::Odometry>& odom) 
  {
    geometry_msgs::PoseStamped pose_in;
    geometry_msgs::PoseStamped pose_out;
    try 
    {
      // Transform the odom reading from the odom frame to the map frame
      //  ie, apply AMCL's localization adjustment
      pose_in.header = odom->header;
      pose_in.pose   = odom->pose.pose;
      tf_.transformPose(target_frame_, pose_in, pose_out);
      
      // Populate the state message
      cutter_msgs::State state_msg;
      state_msg.header.stamp = pose_in.header.stamp;
      state_msg.header.frame_id = target_frame_;
      state_msg.pose.pose = pose_out.pose;
      state_msg.twist.twist = odom->twist.twist;
      ROS_INFO("Pose Found: (%f, %f, %f)",pose_out.pose.position.x, pose_out.pose.position.y, tf::getYaw(pose_out.pose.orientation));
      
      // Publish the message
      state_pub_.publish(state_msg);
    }
    catch (tf::TransformException &ex) 
    {
      ROS_ERROR("Failure %s\n", ex.what()); //Print exception which was caught
    }
    
  };

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "amcl_to_state"); //Init ROS
  AmclToState converter; //Construct class
  ros::spin(); // Run until interupted 
};
