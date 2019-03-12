#include <ros/ros.h>
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


ros::Publisher pub_odom_filtered_ //angular accel_publisher
              ;
std::string pub_odom_topic_name //topic name
          , sub_pose_topic_name
          , sub_twist_topic_name
          , sensor_frame_id
          , robot_frame_id
          , world_frame_id
          ;
nav_msgs::Odometry odom_filtered_;
geometry_msgs::Vector3 sampled_linear_velocity_0 //sample k
                      , sampled_linear_velocity_1 //sample k-1
                      , sampled_angular_velocity_0 //sample k
                      , sampled_angular_velocity_1 //sample k-1s
                      , est_angular_veloc_0 //estimated k
                      , est_angular_veloc_1 //estimated k-1
                      , est_angular_veloc_2 // estimated k-2
                      , est_linear_veloc_0 //estimated k
                      , est_linear_veloc_1 //estimated k-1
                      , est_linear_veloc_2; // estimated k-2

geometry_msgs::Point sampled_linear_pose_0 //sample k
                      , sampled_linear_pose_1 //sample k-1
                      , est_linear_pose_0 //estimated k
                      , est_linear_pose_1 //estimated k-1
                      , est_linear_pose_2; // estimated k-2

geometry_msgs::Quaternion sampled_angular_pose_0 //sample k
                      , sampled_angular_pose_1 //sample k-1
                      , est_angular_pose_0 //estimated k
                      , est_angular_pose_1 //estimated k-1
                      , est_angular_pose_2 // estimated k-2                     
                      ;
double publish_rate = 32.0
      , cutoff_freq_angular = 10.0
      , cutoff_freq_linear = 3.33333
      , damping_angular = 0.96
      , damping_linear = 0.96
      ;

ros::Time sample_time_0, sample_time_1;

/**
* Applies a simple second order filter to two measured samples of the same data
* 
* @param measure - a measure comming from sensor
* @param filtered_0 - the last filtered sample (instant k-1)
* @param filtered_00 - the last filtered sample (instant k-2)
* @param omega_f - cutoff frequency
* @param xi - damping coefficient
* @param dt0 - time interval
*
* @return the new filtered sample
*/
double applyFilter(double measure, double filtered_0, double filtered_00, double omega_f, double xi, double dt0){
    return  pow(dt0,2)*pow(omega_f,2)*measure 
      +(2-2*dt0*omega_f*xi)*filtered_0
      +(2*dt0*omega_f*xi-1-pow(dt0,2)*pow(omega_f,2))*filtered_00;
}
void updatePose(){

  double dt; 
  dt = sample_time_0.toSec()-sample_time_1.toSec(); 

  //Angular velocity estimation
  est_angular_pose_2 = est_angular_pose_1;
  est_angular_pose_1 = est_angular_pose_0;
  est_linear_pose_2 = est_linear_pose_1;
  est_linear_pose_1 = est_linear_pose_0;

  est_angular_pose_0.x = applyFilter(sampled_angular_pose_0.x, 
                                      est_angular_pose_1.x, 
                                      est_angular_pose_2.x,  
                                      cutoff_freq_angular, 
                                      damping_angular, 
                                      dt);
  est_angular_pose_0.y = applyFilter(sampled_angular_pose_0.y, 
                                      est_angular_pose_1.y, 
                                      est_angular_pose_2.y,  
                                      cutoff_freq_angular, 
                                      damping_angular, 
                                      dt);
  est_angular_pose_0.z = applyFilter(sampled_angular_pose_0.z, 
                                      est_angular_pose_1.z, 
                                      est_angular_pose_2.z,  
                                      cutoff_freq_angular, 
                                      damping_angular, 
                                      dt);
  est_angular_pose_0.w = applyFilter(sampled_angular_pose_0.w, 
                                      est_angular_pose_1.w, 
                                      est_angular_pose_2.w,  
                                      cutoff_freq_angular, 
                                      damping_angular, 
                                      dt);

  est_linear_pose_0.x = applyFilter(sampled_linear_pose_0.x, 
                                      est_linear_pose_1.x, 
                                      est_linear_pose_2.x,  
                                      cutoff_freq_linear, 
                                      damping_linear, 
                                      dt);
  est_linear_pose_0.y = applyFilter(sampled_linear_pose_0.y, 
                                      est_linear_pose_1.y, 
                                      est_linear_pose_2.y,  
                                      cutoff_freq_linear, 
                                      damping_linear, 
                                      dt);
  est_linear_pose_0.z = applyFilter(sampled_linear_pose_0.z, 
                                      est_linear_pose_1.z, 
                                      est_linear_pose_2.z,  
                                      cutoff_freq_linear, 
                                      damping_linear, 
                                      dt);

  odom_filtered_.pose.pose.orientation = est_angular_pose_0;
  odom_filtered_.pose.pose.position = est_linear_pose_0;
}
void updateTwist(){

  double dt, dW; 
  dt = sample_time_0.toSec()-sample_time_1.toSec(); 

  //Angular velocity estimation
  est_angular_veloc_2 = est_angular_veloc_1;
  est_angular_veloc_1 = est_angular_veloc_0;
  est_linear_veloc_2 = est_linear_veloc_1;
  est_linear_veloc_1 = est_linear_veloc_0;

  est_angular_veloc_0.x = applyFilter(sampled_angular_velocity_0.x, 
                                      est_angular_veloc_1.x, 
                                      est_angular_veloc_2.x,  
                                      cutoff_freq_angular, 
                                      damping_angular, 
                                      dt);
  est_angular_veloc_0.y = applyFilter(sampled_angular_velocity_0.y, 
                                      est_angular_veloc_1.y, 
                                      est_angular_veloc_2.y,  
                                      cutoff_freq_angular, 
                                      damping_angular, 
                                      dt);
  est_angular_veloc_0.z = applyFilter(sampled_angular_velocity_0.z, 
                                      est_angular_veloc_1.z, 
                                      est_angular_veloc_2.z,  
                                      cutoff_freq_angular, 
                                      damping_angular, 
                                      dt);

  est_linear_veloc_0.x = applyFilter(sampled_linear_velocity_0.x, 
                                      est_linear_veloc_1.x, 
                                      est_linear_veloc_2.x,  
                                      cutoff_freq_linear, 
                                      damping_linear, 
                                      dt);
  est_linear_veloc_0.y = applyFilter(sampled_linear_velocity_0.y, 
                                      est_linear_veloc_1.y, 
                                      est_linear_veloc_2.y,  
                                      cutoff_freq_linear, 
                                      damping_linear, 
                                      dt);
  est_linear_veloc_0.z = applyFilter(sampled_linear_velocity_0.z, 
                                      est_linear_veloc_1.z, 
                                      est_linear_veloc_2.z,  
                                      cutoff_freq_linear, 
                                      damping_linear, 
                                      dt);

  odom_filtered_.twist.twist.angular = est_angular_veloc_0;
  odom_filtered_.twist.twist.linear = est_linear_veloc_0;

  updatePose();

  odom_filtered_.header.frame_id = world_frame_id;
  odom_filtered_.child_frame_id = robot_frame_id;
  odom_filtered_.header.stamp = sample_time_0;
  pub_odom_filtered_.publish(odom_filtered_);

}

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    sampled_angular_pose_1 = sampled_angular_pose_0;
    sampled_linear_pose_1 = sampled_linear_pose_0;
    sampled_angular_pose_0 = pose_msg->pose.pose.orientation;
    sampled_linear_pose_0 = pose_msg->pose.pose.position;
}
void twistCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& twist_msg)
{
  if(twist_msg->header.stamp.toSec() - sample_time_0.toSec() > 1.0/publish_rate){
    sampled_angular_velocity_1 = sampled_angular_velocity_0;
    sampled_linear_velocity_1 = sampled_linear_velocity_0;

    sample_time_1 = sample_time_0;
    sample_time_0 = twist_msg->header.stamp;
    sampled_angular_velocity_0 = twist_msg->twist.twist.angular;
    sampled_linear_velocity_0 = twist_msg->twist.twist.linear;
    updateTwist();

    geometry_msgs::PoseStamped filtered_pose_stamped;
    tf::Stamped<tf::Pose> tf_world_2_base_link;

    filtered_pose_stamped.pose = odom_filtered_.pose.pose;
    filtered_pose_stamped.header = odom_filtered_.header;

    static tf::TransformBroadcaster br;
    tf::poseStampedMsgToTF(filtered_pose_stamped, tf_world_2_base_link);
    br.sendTransform(tf::StampedTransform(tf_world_2_base_link, sample_time_0, world_frame_id, robot_frame_id));

  } 
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "lpf");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nhPrivate("~");
  ros::NodeHandle nh;

  nhPrivate.param<std::string>("pub_odom_topic_name", pub_odom_topic_name, "/odometry/filtered");
  nhPrivate.param<std::string>("sub_twist_topic_name", sub_twist_topic_name, "/droni_sensors/twist_cov");
  nhPrivate.param<std::string>("sub_pose_topic_name", sub_pose_topic_name, "/droni_sensors/pose_cov");
  nhPrivate.param<double>("publish_rate", publish_rate, 32.0);
  nhPrivate.param<std::string>("robot_frame_id", robot_frame_id, "base_link");
  nhPrivate.param<std::string>("world_frame_id", world_frame_id, "world");


  if(nhPrivate.hasParam("cutoff_freq_linear"))
    nhPrivate.getParam("cutoff_freq_linear", cutoff_freq_linear);
  if(nhPrivate.hasParam("damping_linear"))
    nhPrivate.getParam("damping_linear", damping_linear);
  if(nhPrivate.hasParam("cutoff_freq_angular"))
    nhPrivate.getParam("cutoff_freq_angular", cutoff_freq_angular);
  if(nhPrivate.hasParam("damping_angular"))
    nhPrivate.getParam("damping_angular", damping_angular);

  ros::Subscriber sub_twist = nh.subscribe(sub_twist_topic_name, 10, twistCallback);
  ros::Subscriber sub_pose = nh.subscribe(sub_pose_topic_name, 10, poseCallback);

  pub_odom_filtered_ = nh.advertise<nav_msgs::Odometry>(pub_odom_topic_name, 10);


  //Initialize variables
  sampled_angular_velocity_0.x=(0.0); sampled_angular_velocity_0.y=(0.0); sampled_angular_velocity_0.z=(0.0);
  sampled_angular_velocity_1.x=(0.0); sampled_angular_velocity_1.y=(0.0); sampled_angular_velocity_1.z=(0.0);
  est_angular_veloc_0.x=(0.0); est_angular_veloc_0.y=(0.0); est_angular_veloc_0.z=(0.0);
  est_angular_veloc_1.x=(0.0); est_angular_veloc_1.y=(0.0); est_angular_veloc_1.z=(0.0);
  est_angular_veloc_2.x=(0.0); est_angular_veloc_2.y=(0.0); est_angular_veloc_2.z=(0.0);
  sampled_linear_velocity_0.x=(0.0); sampled_linear_velocity_0.y=(0.0); sampled_linear_velocity_0.z=(0.0);
  sampled_linear_velocity_1.x=(0.0); sampled_linear_velocity_1.y=(0.0); sampled_linear_velocity_1.z=(0.0);
  est_linear_veloc_0.x=(0.0); est_linear_veloc_0.y=(0.0); est_linear_veloc_0.z=(0.0);
  est_linear_veloc_1.x=(0.0); est_linear_veloc_1.y=(0.0); est_linear_veloc_1.z=(0.0);
  est_linear_veloc_2.x=(0.0); est_linear_veloc_2.y=(0.0); est_linear_veloc_2.z=(0.0);
  ros::spin();

  return 0;
}