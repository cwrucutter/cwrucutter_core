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
#include "gps_common/GPSStatus.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "cutter_amcl/amclConfig.h"

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
static const std::string gps_status_topic_ = "gps_status";

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
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif
    // Message callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
//    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void gpsReceived(const geometry_msgs::PoseStampedConstPtr& msg);
    void gpsStatusReceived(const gps_common::GPSStatus& status);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
//    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

//  void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
//  void freeMapDependentMemory();
//  map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );
    void applyInitialPose();

    double getYaw(tf::Pose& t);

    //parameter for what odom to use
    std::string odom_frame_id_;
    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

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
    std::vector< AMCLGps* > gps_vec_;
//    std::vector< bool > gps_update_;
    bool gps_update_;
    std::map< std::string, int > frame_to_gps_;

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
    ros::Subscriber gps_status_sub_;

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<cutter_amcl::amclConfig> *dsrv_;
    cutter_amcl::amclConfig default_config_;
    ros::Timer check_gps_timer_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double sigma_gps_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    gps_model_t gps_model_type_;
    double mult_med_noise_;
    double mult_big_noise_;
    int gps_position_source_;

    void reconfigureCB(cutter_amcl::amclConfig &config, uint32_t level);

    ros::Time last_gps_received_ts_;
    ros::Duration gps_check_interval_;
    void checkGpsReceived(const ros::TimerEvent& event);
};

std::vector<std::pair<int,int> > AmclNode::free_space_indices;

#define USAGE "USAGE: amcl"

int
main(int argc, char** argv)
{
  printf("Entering MAIN\n");
  ros::init(argc, argv, "amcl_gps");
  ros::NodeHandle nh;

  AmclNode an;

  ros::spin();

  // To quote Morgan, Hooray!
  return(0);
}

AmclNode::AmclNode() :
        sent_first_transform_(false),
        latest_tf_valid_(false),
        pf_(NULL),
        resample_count_(0),
        odom_(NULL),
        gps_(NULL),
	      private_nh_("~"),
        initial_pose_hyp_(NULL),
        first_map_received_(false),
        first_reconfigure_call_(true)
{
  printf("Entering AmclNode\n");
  printf("%i \n", __LINE__);
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Grab params off the param server
  private_nh_.param("use_map_topic", use_map_topic_, false);
  private_nh_.param("first_map_only", first_map_only_, false);
  printf("%i \n", __LINE__);

  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_rate", tmp, 0.5);
  save_pose_period = ros::Duration(1.0/tmp);
  printf("%i \n", __LINE__);

  private_nh_.param("min_particles", min_particles_, 100);
  private_nh_.param("max_particles", max_particles_, 5000);
  private_nh_.param("kld_err", pf_err_, 0.01);
  private_nh_.param("kld_z", pf_z_, 0.99);
  private_nh_.param("odom_alpha1", alpha1_, 0.2);
  private_nh_.param("odom_alpha2", alpha2_, 0.2);
  private_nh_.param("odom_alpha3", alpha3_, 0.2);
  private_nh_.param("odom_alpha4", alpha4_, 0.2);
  private_nh_.param("odom_alpha5", alpha5_, 0.2);
  printf("%i \n", __LINE__);

  private_nh_.param("gps_sigma", sigma_gps_, 0.01);
  std::string tmp_model_type;
  private_nh_.param("gps_model_type", tmp_model_type, std::string("leverarm"));
  private_nh_.param("mult_med_noise", mult_med_noise_, 3.0);
  private_nh_.param("mult_big_noise", mult_big_noise_, 5.0);
  if(tmp_model_type == "leverarm")
    gps_model_type_ = GPS_MODEL_LEVERARM;
  else
  {
    ROS_WARN("Unknown gps model type \"%s\"; defaulting to leverarm model",
             tmp_model_type.c_str());
    gps_model_type_ = GPS_MODEL_LEVERARM;
  }
  printf("%i \n", __LINE__);

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
  printf("%i \n", __LINE__);

  private_nh_.param("update_min_d", d_thresh_, 0.1);
  private_nh_.param("update_min_a", a_thresh_, M_PI/6.0);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("resample_interval", resample_interval_, 2);
  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.1);
  private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
  private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);
  printf("%i \n", __LINE__);

  transform_tolerance_.fromSec(tmp_tol);
  
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
  printf("%i \n", __LINE__);

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new tf::TransformListener();

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2);
  global_loc_srv_ = nh_.advertiseService("global_localization", 
					 &AmclNode::globalLocalizationCallback,
                                         this);
  //gps_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh_, gps_topic_, 100);
  //gps_filter_ = 
  //        new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*gps_sub_, 
  //                                                                        *tf_, 
  //                                                                        odom_frame_id_, 
  //                                                                        100);
  //gps_filter_->registerCallback(boost::bind(&AmclNode::gpsReceived,
  //                                                 this, _1));
  
  printf("%i \n", __LINE__);
  
/*
#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices.resize(0);
  for(int i = 0; i < map_->size_x; i++)
    for(int j = 0; j < map_->size_y; j++)
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i,j));
#endif*/
  printf("%i \n", __LINE__);
  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)&last_gps_pose_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  printf("%i \n", __LINE__);
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
  
  printf("%i \n", __LINE__);
  
  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  if(odom_model_type_ == ODOM_MODEL_OMNI)
    odom_->SetModelOmni(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);
  else
    odom_->SetModelDiff(alpha1_, alpha2_, alpha3_, alpha4_);
  // Laser
  delete gps_;
  gps_ = new AMCLGps();
  ROS_ASSERT(gps_);
  if(gps_model_type_ == GPS_MODEL_LEVERARM)
    gps_->SetModelLeverarm(sigma_gps_);
  ROS_INFO("Configured GPS and Odom Sensors");
  
  gps_sub_ = nh_.subscribe(gps_topic_,2,&AmclNode::gpsReceived, this);
  gps_status_sub_ = nh_.subscribe(gps_status_topic_,2,&AmclNode::gpsStatusReceived, this);
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
  
  printf("%i \n", __LINE__);
/*
  if(use_map_topic_) {
    map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);
    ROS_INFO("Subscribed to map topic.");
  } else {
    requestMap();
  }
*/
  dsrv_ = new dynamic_reconfigure::Server<cutter_amcl::amclConfig>(ros::NodeHandle("~"));
  dynamic_reconfigure::Server<cutter_amcl::amclConfig>::CallbackType cb = boost::bind(&AmclNode::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  printf("%i \n", __LINE__);

  // 10s timer to warn on lack of receipt of laser scans, #5209
  gps_check_interval_ = ros::Duration(10.0);
  check_gps_timer_ = nh_.createTimer(gps_check_interval_, 
                                       boost::bind(&AmclNode::checkGpsReceived, this, _1));
  printf("Leaving AmclNode\n");
}

void AmclNode::reconfigureCB(cutter_amcl::amclConfig &config, uint32_t level)
{
  printf("Entering reconfigureCB\n");
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  //we don't want to do anything on the first call
  //which corresponds to startup
  if(first_reconfigure_call_)
  {
    first_reconfigure_call_ = false;
    default_config_ = config;
    printf("first_reconfigure_call_ == true, Leaving reconfigureCB\n");
    return;
  }

  if(config.restore_defaults) {
    config = default_config_;
    //avoid looping
    config.restore_defaults = false;
  }

  d_thresh_ = config.update_min_d;
  a_thresh_ = config.update_min_a;

  resample_interval_ = config.resample_interval;

  gui_publish_period = ros::Duration(1.0/config.gui_publish_rate);
  save_pose_period = ros::Duration(1.0/config.save_pose_rate);

  transform_tolerance_.fromSec(config.transform_tolerance);

  alpha1_ = config.odom_alpha1;
  alpha2_ = config.odom_alpha2;
  alpha3_ = config.odom_alpha3;
  alpha4_ = config.odom_alpha4;
  alpha5_ = config.odom_alpha5;

  sigma_gps_ = config.gps_sigma;

  if(config.gps_model_type == "leverarm")
    gps_model_type_ = GPS_MODEL_LEVERARM;

  if(config.odom_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(config.odom_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;

  if(config.min_particles > config.max_particles)
  {
    ROS_WARN("You've set min_particles to be less than max particles, this isn't allowed so they'll be set to be equal.");
    config.max_particles = config.min_particles;
  }

  min_particles_ = config.min_particles;
  max_particles_ = config.max_particles;
  alpha_slow_ = config.recovery_alpha_slow;
  alpha_fast_ = config.recovery_alpha_fast;

  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)&last_gps_pose_);
  pf_err_ = config.kld_err; 
  pf_z_ = config.kld_z; 
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
  pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
  pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose.pose.pose.orientation);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6*0+0];
  pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6*1+1];
  pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6*5+5];
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
  // Laser
  delete gps_;
  gps_ = new AMCLGps();
  ROS_ASSERT(gps_);
  if(gps_model_type_ == GPS_MODEL_LEVERARM)
    gps_->SetModelLeverarm(sigma_gps_);
  ROS_INFO("Configured GPS and Odom Sensors");

  odom_frame_id_ = config.odom_frame_id;
  base_frame_id_ = config.base_frame_id;
  global_frame_id_ = config.global_frame_id;

  /*
  delete gps_filter_;
  gps_filter_ = 
          new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*gps_sub_, 
                                                                          *tf_, 
                                                                          odom_frame_id_, 
                                                                          100);
  gps_filter_->registerCallback(boost::bind(&AmclNode::gpsReceived,
                                            this, _1)); */
  
  gps_sub_ = nh_.subscribe(gps_topic_,2,&AmclNode::gpsReceived, this);
  gps_status_sub_ = nh_.subscribe(gps_status_topic_,2,&AmclNode::gpsStatusReceived, this);
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
  printf("Leaving reconfigureCB\n");
}

void 
AmclNode::checkGpsReceived(const ros::TimerEvent& event)
{
  printf("Entering checkGpsReceived\n");
  ros::Duration d = ros::Time::now() - last_gps_received_ts_;
  if(d > gps_check_interval_)
  {
    ROS_WARN("No gps received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
             d.toSec(),
             ros::names::resolve(gps_topic_).c_str());
  }
  printf("Leaving checkGpsReceived\n");
}

/*
void
AmclNode::requestMap()
{
  boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage( resp.map );
}


void
AmclNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if( first_map_only_ && first_map_received_ ) {
    return;
  }

  handleMapMessage( *msg );

  first_map_received_ = true;
}

void
AmclNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);

  freeMapDependentMemory();
  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();

  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices.resize(0);
  for(int i = 0; i < map_->size_x; i++)
    for(int j = 0; j < map_->size_y; j++)
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i,j));
#endif
  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
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
  // Laser
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  ROS_ASSERT(laser_);
  if(laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_, 0.0);
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }

  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
  applyInitialPose();

}

void
AmclNode::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
  if( pf_ != NULL ) {
    pf_free( pf_ );
    pf_ = NULL;
  }
  delete odom_;
  odom_ = NULL;
  delete laser_;
  laser_ = NULL;
}
*/

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates a map_t and returns it.
 */  /*
map_t*
AmclNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}
*/


AmclNode::~AmclNode()
{
  printf("Entering ~AmclNode\n");
  
  delete dsrv_;
  delete tfb_;
  delete tf_;
  printf("Leaving ~AmclNode\n");
  // TODO: delete everything allocated in constructor
}

bool
AmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
  printf("Entering getOdomPose\n");
  // Get the robot's pose
  ROS_INFO("Time Now: %f, Gps Time Stamp: %f", ros::Time::now().toSec(), t.toSec());
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
  ROS_INFO("Odom pose: x: %f, y: %f, yaw: %f",x,y,yaw);

  printf("Leaving getOdomPose\n");
  return true;
}


pf_vector_t
AmclNode::uniformPoseGenerator(void* arg)
{
  // Sample uniformly around the last gps Pose!
  printf("Entering uniformPoseGenerator\n");
  geometry_msgs::PoseStamped* gps = (geometry_msgs::PoseStamped*)arg;
  
  double min_range, max_range, r, tht;
  min_range = 0.0;
  max_range = 0.3;
  
  pf_vector_t p;
  r = drand48() * (max_range-min_range) + min_range;
  tht = drand48() * (2*M_PI);
  p.v[0] = gps->pose.position.x + r*cos(tht);
  p.v[1] = gps->pose.position.y + r*sin(tht);
  p.v[2] = tht;
  
  /*
  double min_x, max_x, min_y, max_y;
  min_x = -10; 
  min_y = -10;
  max_x = 10;
  max_y = 10;


  pf_vector_t p;

  ROS_INFO("Generating new uniform sample");
  p.v[0] = min_x + drand48() * (max_x - min_x);
  p.v[1] = min_y + drand48() * (max_y - min_y);
  p.v[2] = drand48() * 2 * M_PI - M_PI;*/
  printf("Leaving uniformPoseGenerator\n"); 
  return p;
}

bool
AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  printf("Entering globalLocalizationCallback\n");
  if( map_ == NULL ) {
    printf("Map == null, Leaving globalLocalizationCallback\n");
    return true;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                (void *)&last_gps_pose_);
  ROS_INFO("Global initialisation done!");
  pf_init_ = false;
  printf("Leaving globalLocalizationCallback\n");
  return true;
}

void AmclNode::gpsStatusReceived(const gps_common::GPSStatus& status)
{
  ROS_INFO("GPS Status: %i", status.position_source);
  gps_position_source_ = status.position_source;
}

void
AmclNode::gpsReceived(const geometry_msgs::PoseStampedConstPtr& gps)
{
  double startTime = ros::Time::now().toSec();
  
  // INSERTED FROM HERE... 
  geometry_msgs::PoseStamped gps_tf;
  try
  {
    this->tf_->transformPose(global_frame_id_, ros::Time(gps->header.stamp)-ros::Duration(0.2),
                         (*gps), gps->header.frame_id, gps_tf);
    ROS_INFO("GPS Map: %f, %f    GPS Snowmap: %f, %f", gps->pose.position.x, gps->pose.position.y,
                                                       gps_tf.pose.position.x, gps_tf.pose.position.y);
  }
  catch(tf::TransformException& e)
  {
      ROS_ERROR("Couldn't transform GPS from %s to %s. Skipping GPS message ",
                gps->header.frame_id.c_str(),
                global_frame_id_.c_str());
      printf("Leaving gpsReceived\n");
      return;
  }
  // .... TO HERE
  
  printf("Entering gpsReceived\n");
  last_gps_received_ts_ = ros::Time::now();
  //last_gps_pose_ = *gps;
  last_gps_pose_ = gps_tf;
  
  /*if( map_ == NULL ) {
    printf("Map == null, Leaving gpsReceived\n");
    return;
  }*/
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);
  int gps_index = -1;

  // Do we have the base->base_laser Tx yet?
  if(frame_to_gps_.find(last_gps_pose_.header.frame_id) == frame_to_gps_.end())
  {
    printf("If 1\n");
    ROS_INFO("Setting up gps %d (frame_id=%s)\n", (int)frame_to_gps_.size(), last_gps_pose_.header.frame_id.c_str());
    gps_update_ = true;

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

    // Index each GPS by its frame_id (ie, /base_gps_1, /base_gps_2)
    frame_to_gps_[last_gps_pose_.header.frame_id] = gps_index;
    
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    globalLocalizationCallback(req, res);
    
  } else {
    // we have the gps pose, retrieve laser index
    gps_index = frame_to_gps_[last_gps_pose_.header.frame_id];
  }

  // Where was the robot when this gps measurement was taken?
  tf::Stamped<tf::Pose> odom_pose;
  pf_vector_t pose;
  if(!getOdomPose(odom_pose, pose.v[0], pose.v[1], pose.v[2],
                  last_gps_pose_.header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with gps measurement");
    printf("Leaving gpsReceived\n");
    return;
  }


  pf_vector_t delta = pf_vector_zero();

  if(pf_init_)
  {
    printf("If 2\n");
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
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
     // for(unsigned int i=0; i < gps_update_.size(); i++)
     //   gps_update_[i] = true;
      gps_update_ = true;
      ROS_INFO("Updating now!");
    }
    else
    {
      ROS_INFO("Update not needed");
    }
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    printf("If 3a\n");
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Should update sensor data
    //for(unsigned int i=0; i < gps_update_.size(); i++)
    //  gps_update_[i] = true;
    gps_update_ = true;

    force_publication = true;

    resample_count_ = 0;
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && gps_update_)// gps_update_[gps_index])
  {
    printf("If 3b\n");
    //printf("pose\n");
    //pf_vector_fprintf(pose, stdout, "%.3f");

    AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;

    // Use the action data to update the filter
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

    // Pose at last filter update
    //this->pf_odom_pose = pose;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if(gps_update_) //gps_update_[gps_index])
  {
    AMCLGpsData gdata;;
    gdata.sensor = gps_; //gps_vec_[gps_index];
    gdata.x = last_gps_pose_.pose.position.x;
    gdata.y = last_gps_pose_.pose.position.y;

    //gps_vec_[gps_index]->UpdateSensor(pf_, (AMCLSensorData*)&gdata);
    AMCLSensorData* tempdata = &gdata;
    bool gps_locked = true;
    switch (gps_position_source_) //TODO: subscribe to gps status, grap status flag to local private var. in CB
    {
      case 34:
      case 18:
        ROS_INFO("Narrow Float or WAAS!");
        gps_->SetModelLeverarm(sigma_gps_*mult_med_noise_);
	    break;
      case 17:
      case 16:
        ROS_INFO("Acquiring or Single Pt Solution!");
        gps_->SetModelLeverarm(sigma_gps_*mult_big_noise_);
	    break;
      case 50:
        ROS_INFO("Narrow Int!");
        gps_->SetModelLeverarm(sigma_gps_);
      break;
      default:
        gps_->SetModelLeverarm(sigma_gps_*100);
        gps_locked = false;
        ROS_ERROR("GPS Status (%i) not recognized. Not using GPS", gps_position_source_);
	    break;
    }
    //if (gps_locked)
    //{
      gps_->UpdateSensor(pf_,tempdata);
    //}
    
    //gps_update_[gps_index] = false;
    gps_update_ = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if(!(++resample_count_ % resample_interval_))
    {
      printf("If 4resample\n");
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

  if(resampled || force_publication)
  {
    printf("If 5\n");
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
      printf("If 5maxweight\n");
      ROS_INFO("Max weight pose: %.3f %.3f %.3f",
                hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1],
                hyps[max_weight_hyp].pf_pose_mean.v[2]);

      /*
         puts("");
         pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
         puts("");
       */

      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = last_gps_pose_.header.stamp;
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

      /*
         printf("cov:\n");
         for(int i=0; i<6; i++)
         {
         for(int j=0; j<6; j++)
         printf("%6.3f ", p.covariance[6*i+j]);
         puts("");
         }
       */

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
                                              last_gps_pose_.header.stamp,
                                              base_frame_id_);
        this->tf_->transformPose(odom_frame_id_,
                                 tmp_tf_stamped,
                                 odom_to_map);
      }
      catch(tf::TransformException)
      {
        ROS_INFO("Failed to subtract base to odom transform");
        printf("Leaving gpsReceived\n");
        return;
      }

      latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()));
      latest_tf_valid_ = true;

      // We want to send a transform that is good up until a
      // tolerance time so that odom can be used
      ros::Time transform_expiration = (last_gps_pose_.header.stamp +
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
    printf("If 5else\n");
    // Nothing changed, so we'll just republish the last transform, to keep
    // everybody happy.
    ros::Time transform_expiration = (last_gps_pose_.header.stamp +
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
  printf("Leaving gpsReceived\n");

}

double
AmclNode::getYaw(tf::Pose& t)
{
  printf("Entering getYaw\n");
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  printf("Leaving getYaw\n");
  return yaw;
}

void
AmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
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
void
AmclNode::applyInitialPose()
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
