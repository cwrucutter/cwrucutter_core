#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "cutter_msgs/StateEvaluation.h"

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



class LocalizationEvaluation
{
public:
  LocalizationEvaluation() : tf_(),  target_frame_("base_gps")
  {
    dgps_sub_.subscribe(nh_, "gps_pose", 10);
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(dgps_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&LocalizationEvaluation::msgCallback, this, _1) );
    err_pub_ = nh_.advertise<cutter_msgs::StateEvaluation>("cwru/localization_err",1);
  } ;

private:
  message_filters::Subscriber<geometry_msgs::PoseStamped> dgps_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<geometry_msgs::PoseStamped> * tf_filter_;
  ros::Publisher err_pub_;
  ros::NodeHandle nh_;
  std::string target_frame_;
  RunningStat stat_;

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& pose) 
  {
    geometry_msgs::PoseStamped pose_err;
    double mag_err;
    try 
    {
      tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)),
                                 ros::Time(), "base_gps"); //TODO: replace base_gps with a configurable value
      tf::Stamped<tf::Pose> est_pose;
      tf_.transformPose("map", ident, est_pose);
      printf("Esimated GPS pose?? (x:%f y:%f z:%f)\n", 
             est_pose.getOrigin().x(),
             est_pose.getOrigin().y(),
             est_pose.getOrigin().z());
             
      printf("Measured GPS pose?? (x:%f y:%f z:%f)\n", 
             pose->pose.position.x,
             pose->pose.position.y,
             pose->pose.position.z);
    
      tf_.transformPose(target_frame_, *pose, pose_err);
      mag_err = sqrt(pose_err.pose.position.x*pose_err.pose.position.x + pose_err.pose.position.y*pose_err.pose.position.y);
      
      printf("gps_pose in the base_gps frame?? (x:%f y:%f z:%f)\n", 
             pose_err.pose.position.x,
             pose_err.pose.position.y,
             pose_err.pose.position.z);
      printf("Magnitude error sqrt(x^2+y^2): %f\n", mag_err);
      stat_.Push(mag_err);
      printf("Mean: %f, Variance: %f, Std Dev: %f\n", stat_.Mean(), stat_.Variance(), stat_.StandardDeviation());
      
      cutter_msgs::StateEvaluation err_msg;
      err_msg.x_err = pose_err.pose.position.x*pose_err.pose.position.x;
      err_msg.y_err = pose_err.pose.position.y*pose_err.pose.position.y;
      err_msg.tht_err = 0.0; // Currently dont know?? hmmm...
      err_msg.xy_err = mag_err;
      err_msg.xy_err_mean = stat_.Mean();
      err_msg.xy_err_variance = stat_.Variance();
      err_msg.xy_err_stddev = stat_.StandardDeviation();
      
      err_pub_.publish(err_msg);
    }
    catch (tf::TransformException &ex) 
    {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
    
  };

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "localization_evaluation"); //Init ROS
  LocalizationEvaluation eval; //Construct class
  ros::spin(); // Run until interupted 
};
