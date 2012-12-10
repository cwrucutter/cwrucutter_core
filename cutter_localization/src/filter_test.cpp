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
 * localization_evaluation.cpp
 *   Evaluates the localization solution by comparing to the differential GPS measurement
 * 
 * Subscribes:
 *   - gps_pose (geometry_msgs/PoseStamped): Current differential GPS reading
 *  
 * Publishes:
 *   - localization_err (cutter_msgs/StateEvaluaton): Current Error stats for the localization solution
 *
 ********************************************************************************/

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "cutter_msgs/StateEvaluation.h"

class FilterTest
{
public:
  FilterTest() : tf_(),  target_frame_("odom")
  {
    gps_sub_.subscribe(nh_, "gps_pose", 10);
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(gps_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&FilterTest::msgCallback, this, _1) );
  } ;

private:
  message_filters::Subscriber<geometry_msgs::PoseStamped> gps_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<geometry_msgs::PoseStamped> * tf_filter_;
  ros::NodeHandle nh_;
  std::string target_frame_;

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& pose) 
  {
    printf("Measured GPS pose?? (x:%f y:%f z:%f)\n", 
           pose->pose.position.x,
           pose->pose.position.y,
           pose->pose.position.z);
  };

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "filter_test"); //Init ROS
  FilterTest eval; //Construct class
  ros::spin(); // Run until interupted 
};
