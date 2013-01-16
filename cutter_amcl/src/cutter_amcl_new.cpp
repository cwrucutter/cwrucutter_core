/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */
/* Modified by EJ Kreinar for GPS localization */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

#include "map/map.h"
#include "pf/pf.h"
#include "sensors/amcl_odom.h"
#include "sensors/amcl_gps.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

#define NEW_UNIFORM_SAMPLING 1

using namespace amcl;

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

//static const std::string scan_topic_ = "scan";
static const std::string gps_topic_ = "gps_pose";

class AmclNode
{
  public:
    AmclNode();
    ~AmclNode();

    int process();

  private:
    tf::TransformBroadcaster* tfb_;
    tf::TransformListener* tf_;

    bool sent_first_transform_;

    tf::Transform latest_tf_;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);
    
    // Message callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    void gpsReceived(const geometry_msgs::PoseStampedConstPtr& msg);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

    void applyInitialPose();

    double getYaw(tf::Pose& t);

    //parameter for what odom to use
    std::string odom_frame_id_;
    //parameter for what base to use
    std::string base_frame_id_;
    //parameter for what global frame to use
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;
    bool use_gps_;
    bool use_beacon_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;
    geometry_msgs::PoseStamped last_gps_pose_;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    //message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* gps_sub_;
    //tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* gps_filter_;
    ros::Subscriber gps_sub_;
    ros::Subscriber initial_pose_sub_;
    bool pf_update_;
    bool gps_initialized_;

    // Particle filter
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;

    AMCLOdom* odom_;
    AMCLGps* gps_;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_;
    ros::ServiceServer global_loc_srv_;
    ros::Subscriber initial_pose_sub_old_;
    ros::Subscriber map_sub_;

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    ros::Timer check_gps_timer_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double sigma_gps_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    gps_model_t gps_model_type_;

    ros::Time last_gps_received_ts_;
    ros::Time last_amcl_ts_;
    ros::Duration gps_check_interval_;
    void checkGpsReceived(const ros::TimerEvent& event);
};

#define USAGE "USAGE: amcl"

int main(int argc, char** argv)
{
  printf("Entering MAIN\n");
  ros::init(argc, argv, "amcl_gps_new");
  ros::NodeHandle nh;

  AmclNode an;
  
  ros::Rate loop_rate(10.0); // Loop at 10 Hz
  while (ros::ok())
  {
    ros::spinOnce();   // Run callbacks
    an.process();      // Process the Sensors using AMCL
    loop_rate.sleep(); // Sleep at the loop rate
  }
  
  return(0);
}

AmclNode::AmclNode() :
        sent_first_transform_(false),
        latest_tf_valid_(false),
        gps_initialized_(false),
        pf_(NULL),
        resample_count_(0),
        odom_(NULL),
        gps_(NULL),
	      private_nh_("~"),
        initial_pose_hyp_(NULL),
        first_reconfigure_call_(true)
{
  ROS_INFO("Entering AmclNode constructor\n");
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // GET PARAMS
  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_rate", tmp, 0.5);
  save_pose_period = ros::Duration(1.0/tmp);

  // Particle filter and odometry parameters
  private_nh_.param("min_particles", min_particles_, 100);
  private_nh_.param("max_particles", max_particles_, 5000);
  private_nh_.param("kld_err", pf_err_, 0.01);
  private_nh_.param("kld_z", pf_z_, 0.99);
  private_nh_.param("odom_alpha1", alpha1_, 0.2);
  private_nh_.param("odom_alpha2", alpha2_, 0.2);
  private_nh_.param("odom_alpha3", alpha3_, 0.2);
  private_nh_.param("odom_alpha4", alpha4_, 0.2);
  private_nh_.param("odom_alpha5", alpha5_, 0.2);

  // GPS parameters
  private_nh_.param("use_gps", use_gps_, true);
  private_nh_.param("gps_sigma", sigma_gps_, 0.01);
  std::string tmp_model_type;
  private_nh_.param("gps_model_type", tmp_model_type, std::string("leverarm"));
  if(tmp_model_type == "leverarm")
    gps_model_type_ = GPS_MODEL_LEVERARM;
  else
  {
    ROS_WARN("Unknown gps model type \"%s\"; defaulting to leverarm model",
             tmp_model_type.c_str());
    gps_model_type_ = GPS_MODEL_LEVERARM;
  }

  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
  if(tmp_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
             tmp_model_type.c_str());
    odom_model_type_ = ODOM_MODEL_DIFF;
  }
 
  // Beacon parameters
  private_nh_.param("use_beacon", use_beacon_, true);
 
  // More particle filter parameters
  private_nh_.param("update_min_d", d_thresh_, 0.1);
  private_nh_.param("update_min_a", a_thresh_, M_PI/6.0);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("snowmap"));
  private_nh_.param("resample_interval", resample_interval_, 2);
  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.1);
  private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
  private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);
  
  transform_tolerance_.fromSec(tmp_tol);

  // Initial Pose parameters
  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
  
  // Check for NAN on input from param server, #5239
  double tmp_pos;
  private_nh_.param("initial_pose_x", tmp_pos, init_pose_[0]);
  if(!std::isnan(tmp_pos))
    init_pose_[0] = tmp_pos;
  else 
    ROS_WARN("ignoring NAN in initial pose X position");
  private_nh_.param("initial_pose_y", tmp_pos, init_pose_[1]);
  if(!std::isnan(tmp_pos))
    init_pose_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Y position");
  private_nh_.param("initial_pose_a", tmp_pos, init_pose_[2]);
  if(!std::isnan(tmp_pos))
    init_pose_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Yaw");
  private_nh_.param("initial_cov_xx", tmp_pos, init_cov_[0]);
  if(!std::isnan(tmp_pos))
    init_cov_[0] =tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance XX");
  private_nh_.param("initial_cov_yy", tmp_pos, init_cov_[1]);
  if(!std::isnan(tmp_pos))
    init_cov_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance YY");
  private_nh_.param("initial_cov_aa", tmp_pos, init_cov_[2]);
  if(!std::isnan(tmp_pos))
    init_cov_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance AA");
    
  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new tf::TransformListener();

  // Particle Filter Publishers and Localization Service
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
  global_loc_srv_ = nh_.advertiseService("global_localization", 
					 &AmclNode::globalLocalizationCallback,
                                         this);

  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)&last_gps_pose_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = init_pose_[0];
  pf_init_pose_mean.v[1] = init_pose_[1];
  pf_init_pose_mean.v[2] = init_pose_[2];
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = init_cov_[0];
  pf_init_pose_cov.m[1][1] = init_cov_[1];
  pf_init_pose_cov.m[2][2] = init_cov_[2];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;
  
  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  if(odom_model_type_ == ODOM_MODEL_OMNI)
    odom_->SetModelOmni(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  else
    odom_->SetModelDiff(alpha1_, alpha2_, alpha3_, alpha4_);
    
  // GPS
  delete gps_;
  gps_ = new AMCLGps();
  ROS_ASSERT(gps_);
  if(gps_model_type_ == GPS_MODEL_LEVERARM)
    gps_->SetModelLeverarm(sigma_gps_);
  ROS_INFO("Configured GPS and Odom Sensors");
  
  // Subscribers
  gps_sub_ = nh_.subscribe(gps_topic_,2,&AmclNode::gpsReceived, this);
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);

  // 5s timer to warn on lack of receipt of gps data
  gps_check_interval_ = ros::Duration(5.0);
  check_gps_timer_ = nh_.createTimer(gps_check_interval_, boost::bind(&AmclNode::checkGpsReceived, this, _1));
  ROS_INFO("Leaving AmclNode constructor\n");
}

void AmclNode::checkGpsReceived(const ros::TimerEvent& event)
{
  ROS_INFO("Entering checkGpsReceived\n");
  ros::Duration d = ros::Time::now() - last_gps_received_ts_;
  if(d > gps_check_interval_)
  {
    ROS_WARN("No gps received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
             d.toSec(),
             ros::names::resolve(gps_topic_).c_str());
  }
  ROS_INFO("Leaving checkGpsReceived\n");
}

AmclNode::~AmclNode()
{
  ROS_INFO("Entering ~AmclNode destructor\n");
  
  delete tfb_;
  delete tf_;
  ROS_INFO("Leaving ~AmclNode destructor\n");
  // TODO: delete everything allocated in constructor
}

bool AmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                           double& x, double& y, double& yaw,
                           const ros::Time& t, const std::string& f)
{
  //ROS_INFO("Entering getOdomPose\n");
  // Get the robot's pose
  //ROS_INFO("Time Now: %f, Gps Time Stamp: %f", ros::Time::now().toSec(), t.toSec());
  double startTime = ros::Time::now().toSec();
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)), t, f);
  try
  {
    this->tf_->waitForTransform(odom_frame_id_, f, t, ros::Duration(0.1));
    this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    printf("Leaving getOdomPose\n");
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);
  //ROS_INFO("Odom pose: x: %f, y: %f, yaw: %f",x,y,yaw);

  //ROS_INFO("Leaving getOdomPose\n");
  double endTime = ros::Time::now().toSec();
  ROS_INFO("getOdomPose Duration: %f", endTime-startTime);
  return true;
}


pf_vector_t AmclNode::uniformPoseGenerator(void* arg)
{
  // Sample uniformly around the last measurement!!
  //ROS_INFO("Entering uniformPoseGenerator\n");
  geometry_msgs::PoseStamped* gps = (geometry_msgs::PoseStamped*)arg;
  
  double min_range, max_range, r, tht;
  min_range = 0.3;
  max_range = 0.6;
  
  pf_vector_t p;
  r = drand48() * (max_range-min_range) + min_range;
  tht = drand48() * (2*M_PI);
  p.v[0] = gps->pose.position.x + r*cos(tht);
  p.v[1] = gps->pose.position.y + r*sin(tht);
  p.v[2] = tht;
  
  return p;
}

bool AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                          std_srvs::Empty::Response& res)
{
  // Global Localization Server: 
  //    Runs when the service is called. Uses uniformPoseGenerator to create uniform poses
  ROS_INFO("Entering globalLocalizationCallback\n");
  
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                (void *)&last_gps_pose_);
  ROS_INFO("Global initialisation done!");
  pf_init_ = false;
  ROS_INFO("Leaving globalLocalizationCallback\n");
  return true;
}

void AmclNode::gpsReceived(const geometry_msgs::PoseStampedConstPtr& gps)
{ 
  last_gps_received_ts_ = ros::Time::now();
  last_gps_pose_ = *gps;
  
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);

  // Do we have the base->base_gps Tx yet? We need to know the location of 
  //  the gps with respect to the robot origin
  if(!gps_initialized_)
  {
    ROS_INFO("Setting up gps (frame_id=%s)\n", gps->header.frame_id.c_str());
    pf_update_ = true;

    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)),
                                 ros::Time(), "base_gps"); //TODO: replace base_gps with a configurable value
    tf::Stamped<tf::Pose> gps_pose;
    
    try
    {
      this->tf_->transformPose(base_frame_id_, ident, gps_pose);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                "base_gps",
                base_frame_id_.c_str());
      printf("Leaving gpsReceived\n");
      return;
    }

    pf_vector_t gps_pose_v;
    gps_pose_v.v[0] = gps_pose.getOrigin().x();
    gps_pose_v.v[1] = gps_pose.getOrigin().y();
    // laser mounting angle gets computed later -> set to 0 here!
    gps_pose_v.v[2] = 0;
    //gps_vec_[gps_index]->SetGpsPose(gps_pose_v);
    gps_->SetGpsPose(gps_pose_v);
    ROS_INFO("Received gps pose wrt robot: %.3f %.3f %.3f",
              gps_pose_v.v[0],
              gps_pose_v.v[1],
              gps_pose_v.v[2]);

    gps_initialized_ = true;
  } 
  
}


int AmclNode::process()
{
  bool gps_new = (last_gps_received_ts_ - last_amcl_ts_) > ros::Duration(0.0);
  bool beacon_new = true;
  geometry_msgs::PoseStamped gps = last_gps_pose_;
  
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  last_amcl_ts_ = ros::Time::now();
  double startTime = ros::Time::now().toSec();
  
  // 1. GET ODOMETRY
  tf::Stamped<tf::Pose> odom_pose;
  pf_vector_t pose;
  if(!getOdomPose(odom_pose, pose.v[0], pose.v[1], pose.v[2],
                  last_amcl_ts_, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with gps measurement");
    printf("Leaving process\n");
    return 0;
  }

  pf_vector_t delta = pf_vector_zero();
  if(pf_init_)
  {
    // Compute change in pose
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);
    ROS_INFO("Pose Change: x: %f, y: %f, yaw: %f",delta.v[0],delta.v[2],delta.v[2]);

    // See if we should update the filter
    bool update = fabs(delta.v[0]) > d_thresh_ ||
                  fabs(delta.v[1]) > d_thresh_ ||
                  fabs(delta.v[2]) > a_thresh_;
    ROS_INFO("xdiff: %f, ydiff: %f, adiff: %f, dthresh: %f, athresh_ %f", fabs(delta.v[0]),fabs(delta.v[1]),fabs(delta.v[2]),d_thresh_,a_thresh_);

    // Set the laser update flags
    if(update)
    {
      pf_update_ = true;
      ROS_INFO("Updating now!");
    }
    else
    {
      ROS_INFO("Update not needed");
    }
  }

  // 2. PARTICLE FILTER ODOMETRY UPDATE
  // If the robot has moved, update the filter
  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    pf_update_ = true;
    force_publication = true;

    resample_count_ = 0;
  }
  else if(pf_init_ && pf_update_)
  {
    AMCLOdomData odata;
    odata.pose = pose;
    
    // Store the odom pose for next iteration
    pf_odom_pose_ = pose; 
    
    // Set the odometry delta
    odata.delta = delta;

    // Use the action data to update the filter
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);
  }

  // 3. SENSOR UPDATE
  //    Modify the particle weight based on each sensor
  // Sensor 1: GPS
  if(pf_update_ && gps_new && use_gps_)  
  {
    printf("GPS Update\n");
    AMCLGpsData gdata;
    gdata.sensor = gps_; //gps_vec_[gps_index];
    gdata.x = gps.pose.position.x;
    gdata.y = gps.pose.position.y;

    AMCLSensorData* tempdata = &gdata;
    gps_->UpdateSensor(pf_,tempdata); // Update weights based on the GPS
  }
  
  // Sensor 2: Beacon
  if (pf_update_ && beacon_new && use_beacon_)
  {
    printf("Beacon Update\n");
    //AMCLBeaconData bdata; //initialize beacon
    //bdata.sensor = beacon_; // set the sensor
    //bdata.dist = // Set the measurement
    
    //AMCLSensorData* tempdata = &bdata;
    //beacon_->UpdateSensor(pf_,tempdata); // Update weights based on the beacon
    
  }
  
  // 4. RESAMPLING
  bool resampled = false;
  if (pf_update_)
  {
    pf_update_ = false;
    
    // Resample the particles
    if(!(++resample_count_ % resample_interval_))
    {
      pf_update_resample(pf_);
      resampled = true;
    }

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    ROS_INFO("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    // TODO: set maximum rate for publishing
    geometry_msgs::PoseArray cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = global_frame_id_;
    cloud_msg.poses.resize(set->sample_count);
    for(int i=0;i<set->sample_count;i++)
    {
      tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                               tf::Vector3(set->samples[i].pose.v[0],
                                         set->samples[i].pose.v[1], 0)),
                      cloud_msg.poses[i]);

    }

    particlecloud_pub_.publish(cloud_msg);
  }

  // 5. CLEANUP, ORGANIZATION, PUBLISHING STUFF
  if(resampled || force_publication)
  {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for(int hyp_count = 0;
        hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }
      //ROS_INFO("Num Samples: %i, Num Clusters: %i",pf_->sets[pf_->current_set].sample_count,pf_->sets[pf_->current_set].cluster_count);
      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if(max_weight > 0.0)
    {
      ROS_INFO("Max weight pose: %.3f %.3f %.3f",
                hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1],
                hyps[max_weight_hyp].pf_pose_mean.v[2]);

      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = gps.header.stamp;
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                            p.pose.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          // Report the overall filter covariance, rather than the
          // covariance for the highest-weight cluster
          //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
          p.pose.covariance[6*i+j] = set->cov.m[i][j];
        }
      }
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      p.pose.covariance[6*5+5] = set->cov.m[2][2];

      pose_pub_.publish(p);
      last_published_pose = p;

      ROS_INFO("New pose: %6.3f %6.3f %6.3f",
               hyps[max_weight_hyp].pf_pose_mean.v[0],
               hyps[max_weight_hyp].pf_pose_mean.v[1],
               hyps[max_weight_hyp].pf_pose_mean.v[2]);

      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                             tf::Vector3(hyps[max_weight_hyp].pf_pose_mean.v[0],
                                         hyps[max_weight_hyp].pf_pose_mean.v[1],
                                         0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              gps.header.stamp,
                                              base_frame_id_);
        this->tf_->transformPose(odom_frame_id_,
                                 tmp_tf_stamped,
                                 odom_to_map);
      }
      catch(tf::TransformException)
      {
        ROS_INFO("Failed to subtract base to odom transform");
        printf("Leaving process\n");
        return 0;
      }

      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;

      // We want to send a transform that is good up until a
      // tolerance time so that odom can be used
      ros::Time transform_expiration = (gps.header.stamp +
                                        transform_tolerance_);
      tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                          transform_expiration,
                                          global_frame_id_, odom_frame_id_);
      this->tfb_->sendTransform(tmp_tf_stamped);
      sent_first_transform_ = true;
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }
  else if(latest_tf_valid_)
  {
    // Nothing changed, so we'll just republish the last transform, to keep
    // everybody happy.
    ros::Time transform_expiration = (gps.header.stamp +
                                      transform_tolerance_);
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_id_, odom_frame_id_);
    this->tfb_->sendTransform(tmp_tf_stamped);


    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if((save_pose_period.toSec() > 0.0) &&
       (now - save_pose_last_time) >= save_pose_period)
    {
      // We need to apply the last transform to the latest odom pose to get
      // the latest map pose to store.  We'll take the covariance from
      // last_published_pose.
      tf::Pose map_pose = latest_tf_.inverse() * odom_pose;
      double yaw,pitch,roll;
      map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

      private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
      private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
      private_nh_.setParam("initial_pose_a", yaw);
      private_nh_.setParam("initial_cov_xx", 
                                      last_published_pose.pose.covariance[6*0+0]);
      private_nh_.setParam("initial_cov_yy", 
                                      last_published_pose.pose.covariance[6*1+1]);
      private_nh_.setParam("initial_cov_aa", 
                                      last_published_pose.pose.covariance[6*5+5]);
      save_pose_last_time = now;
    }
  }
  double endTime = ros::Time::now().toSec();
  ROS_INFO("Loop Duration: %f", endTime-startTime);
  
  return 1;

}

double AmclNode::getYaw(tf::Pose& t)
{
  printf("Entering getYaw\n");
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  printf("Leaving getYaw\n");
  return yaw;
}

void AmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  printf("Entering initialPoseReceived\n");
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  if(msg->header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if(tf_->resolve(msg->header.frame_id) != tf_->resolve(global_frame_id_))
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             msg->header.frame_id.c_str(),
             global_frame_id_.c_str());
    printf("Leaving initialPoseReceived\n");
    return;
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try
  {
    tf_->lookupTransform(base_frame_id_, ros::Time::now(),
                         base_frame_id_, msg->header.stamp,
                         global_frame_id_, tx_odom);
  }
  catch(tf::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  tf::Pose pose_old, pose_new;
  tf::poseMsgToTF(msg->pose.pose, pose_old);
  pose_new = tx_odom.inverse() * pose_old;

  // Transform into the global frame

  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
           ros::Time::now().toSec(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
           getYaw(pose_new));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = getYaw(pose_new);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      pf_init_pose_cov.m[i][j] = msg->pose.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg->pose.covariance[6*5+5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  applyInitialPose();
  printf("Leaving initialPoseReceived\n");
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void AmclNode::applyInitialPose()
{
  printf("Entering applyInitialPose\n");
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  if( initial_pose_hyp_ != NULL && map_ != NULL ) {
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;

    delete initial_pose_hyp_;
    initial_pose_hyp_ = NULL;
  }
  printf("Leaving applyInitialPose\n");
}
