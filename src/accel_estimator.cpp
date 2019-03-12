#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <vector> // for vector 
#include <iterator> // for back_inserter 

ros::Publisher estimated_accel_pub_ //angular accel_publisher
              , corrected_accel_pub_ //imu/data_corrected publisher
              ;
std::string pub_estimated_accel_topic_name //topic name
          , sub_odom_topic_name
          , sub_imu_data_topic_name
          , pub_corrected_accel_topic_name
          , sensor_frame_id
          , robot_frame_id
          ;

geometry_msgs::AccelStamped filtered_accel;
geometry_msgs::Vector3 sampled_linear_accel //sample k
                      , est_angular_veloc_0 //estimated k
                      , est_angular_veloc_1 //estimated k-1
                      , est_angular_veloc_2 // estimated k-2
                      , est_angular_accel_0
                      , est_angular_accel_1
                      , est_angular_accel_2
                      ;
geometry_msgs::Vector3  sampled_angular_velocity_0 //sample k
                    , sampled_angular_velocity_1; //sample k-1
ros::Time sample_time_0, sample_time_1;
tf::Vector3 gondola_pos;
sensor_msgs::Imu corrected_imu_msg_;


typedef struct fil_gains{
  std::vector<double> linear_x;
  std::vector<double> linear_y;
  std::vector<double> linear_z;
  std::vector<double> angular_x;
  std::vector<double> angular_y;
  std::vector<double> angular_z;
} FilterGains;

FilterGains filter_gains;

tf::StampedTransform tf_sensor_2_robot_;


/**
* Applies a simple second order filter to two measured samples of the same data
* 
* @param measure_1 - a measure comming from sensor
* @param measure_2 - second measure comming from sensor (optional)
* @param filtered_1 - the last filtered sample (instant k-1)
* @param filtered_2 - the last filtered sample (instant k-2)
* @param gains - bandwidth  constant for each measure
*
* @return the new filtered sample
*/
double applyFilter(double measure_1, double measure_2, double filtered_1, double filtered_2, std::vector<double> gains){
  double sum = 0.0;
  for(int i=0;i<gains.size();i++)
    sum +=gains[i];
  
  if(sum>0){
    //normalizing gains, the sum must be 1.0
    if(gains.size()>3){
      return measure_1*gains[0]/sum + measure_2*gains[1]/sum + filtered_1*gains[2]/sum + filtered_2*gains[3]/sum;
    }else{
      return measure_1*gains[0]/sum + filtered_1*gains[1]/sum + filtered_2*gains[2]/sum;
    }
  }else{
    return 0.0;
  }
}

void updateAccel(){

  double dt, dW, dWdt; 
  dt = sample_time_0.toSec()-sample_time_1.toSec(); 

  //Angular velocity estimation
  est_angular_veloc_2 = est_angular_veloc_1;
  est_angular_veloc_1 = est_angular_veloc_0;
  est_angular_accel_2 = est_angular_accel_1;
  est_angular_accel_1 = est_angular_accel_0;


  est_angular_veloc_0.x = applyFilter(sampled_angular_velocity_0.x, 0.0, est_angular_veloc_1.x, est_angular_veloc_2.x,  filter_gains.linear_x);
  dW = est_angular_veloc_0.x-est_angular_veloc_1.x;
  dWdt = (dt>0) ? dW/dt : filtered_accel.accel.angular.x;
  filtered_accel.accel.angular.x = applyFilter(dWdt, 0.0, filtered_accel.accel.angular.x, 0.0,  filter_gains.angular_x);


  est_angular_veloc_0.y = applyFilter(sampled_angular_velocity_0.y, 0.0, est_angular_veloc_1.y, est_angular_veloc_2.y,  filter_gains.linear_y);
  dW = est_angular_veloc_0.y-est_angular_veloc_1.y;
  dWdt = (dt>0) ? dW/dt : filtered_accel.accel.angular.y;
  filtered_accel.accel.angular.y = applyFilter(dWdt, 0.0, filtered_accel.accel.angular.y, 0.0,  filter_gains.angular_y);


  est_angular_veloc_0.z = applyFilter(sampled_angular_velocity_0.z, 0.0, est_angular_veloc_1.z, est_angular_veloc_2.z,  filter_gains.linear_z);
  dW = est_angular_veloc_0.z-est_angular_veloc_1.z;
  dWdt = (dt>0) ? dW/dt : filtered_accel.accel.angular.z;
  filtered_accel.accel.angular.z = applyFilter(dWdt, 0.0, filtered_accel.accel.angular.z, 0.0,  filter_gains.angular_z);


  // filtered_accel.accel.angular = est_angular_veloc_0;
  ROS_INFO("dt:%f", dt);

  ROS_INFO("dWdt:%f", dWdt);
  ROS_INFO("dWdt_f:%f", filtered_accel.accel.angular.z);
  filtered_accel.accel.linear = est_angular_veloc_0;
  filtered_accel.header.frame_id = robot_frame_id;
  filtered_accel.header.stamp = sample_time_0;
  estimated_accel_pub_.publish(filtered_accel);

}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  sampled_angular_velocity_1 = sampled_angular_velocity_0;
  sample_time_1 = sample_time_0;
  sample_time_0 = odom_msg->header.stamp;
  sampled_angular_velocity_0 = odom_msg->twist.twist.angular;
  updateAccel();
}
void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  corrected_imu_msg_ = *imu_msg;
  tf::Vector3 lin_acc, ang_acc, ang_vel;
  tf::vector3MsgToTF(imu_msg->linear_acceleration, lin_acc);
  tf::vector3MsgToTF(filtered_accel.accel.angular, ang_acc);
  tf::vector3MsgToTF(sampled_angular_velocity_0, ang_vel);

  //ROS_INFO_STREAM("gondola_pos x: " << gondola_pos.getX());
  
  lin_acc = lin_acc - ang_acc.cross(gondola_pos)- ang_vel.cross(ang_vel.cross(gondola_pos));

  tf::vector3TFToMsg(lin_acc, corrected_imu_msg_.linear_acceleration);
  corrected_accel_pub_.publish(corrected_imu_msg_);
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
  ros::init(argc, argv, "accel_estimator");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nhPrivate("~");
  ros::NodeHandle nh;

  nhPrivate.param<std::string>("pub_corrected_accel_topic_name", pub_corrected_accel_topic_name, "/droni_sensors/imu/data_corrected");
  nhPrivate.param<std::string>("pub_estimated_accel_topic_name", pub_estimated_accel_topic_name, "/droni_sensors/angular_accel_filtered");
  nhPrivate.param<std::string>("sub_odom_topic_name", sub_odom_topic_name, "/odometry/filtered");
  nhPrivate.param<std::string>("sub_imu_data_topic_name", sub_imu_data_topic_name, "/droni_sensors/imu/data");
  nhPrivate.param<std::string>("robot_frame_id", robot_frame_id, "base_link");
  nhPrivate.param<std::string>("sensor_frame_id", sensor_frame_id, "gondola_link");


  std::vector<double> linear_param(3, 0.1);
  if(nhPrivate.hasParam("filter_gains/linear_x"))
    nhPrivate.getParam("filter_gains/linear_x", linear_param);

  filter_gains.linear_x.assign(linear_param.begin(), linear_param.end());


  if(nhPrivate.hasParam("filter_gains/linear_y"))
    nhPrivate.getParam("filter_gains/linear_y", linear_param);

    filter_gains.linear_y.assign(linear_param.begin(), linear_param.end());


  if(nhPrivate.hasParam("filter_gains/linear_z"))
    nhPrivate.getParam("filter_gains/linear_z", linear_param);

 filter_gains.linear_z.assign(linear_param.begin(), linear_param.end());
 

  std::vector<double> angular_param(3, 0.1);
  if(nhPrivate.hasParam("filter_gains/angular_x"))
    nhPrivate.getParam("filter_gains/angular_x", angular_param);
  
  filter_gains.angular_x.assign(angular_param.begin(), angular_param.end());


  if(nhPrivate.hasParam("filter_gains/angular_y"))
    nhPrivate.getParam("filter_gains/angular_y", angular_param);
  
  filter_gains.angular_y.assign(angular_param.begin(), angular_param.end());


  if(nhPrivate.hasParam("filter_gains/angular_z"))
    nhPrivate.getParam("filter_gains/angular_z", angular_param);
  
  filter_gains.angular_z.assign(angular_param.begin(), angular_param.end());

  std::vector<double> gondvec(3, 0.0);
  if(nhPrivate.hasParam("gondola_pos"))
    nhPrivate.getParam("gondola_pos", gondvec);

  gondola_pos.setX(gondvec[0]);
  gondola_pos.setY(gondvec[1]);
  gondola_pos.setZ(gondvec[2]);

  sampled_angular_velocity_0.x=(0.0); sampled_angular_velocity_0.y=(0.0); sampled_angular_velocity_0.z=(0.0);
  sampled_angular_velocity_0.x=(0.0); sampled_angular_velocity_0.y=(0.0); sampled_angular_velocity_0.z=(0.0);
  est_angular_veloc_0.x=(0.0); est_angular_veloc_0.y=(0.0); est_angular_veloc_0.z=(0.0);
  est_angular_veloc_1.x=(0.0); est_angular_veloc_1.y=(0.0); est_angular_veloc_1.z=(0.0);
  est_angular_veloc_2.x=(0.0); est_angular_veloc_2.y=(0.0); est_angular_veloc_2.z=(0.0);
  sample_time_0 = ros::Time::now();

  ros::Subscriber sub_odom_ = nh.subscribe(sub_odom_topic_name, 10, odomCallback);
  estimated_accel_pub_ = nh.advertise<geometry_msgs::AccelStamped>(pub_estimated_accel_topic_name, 10);

  ros::Subscriber sub_imu_data_ = nh.subscribe(sub_imu_data_topic_name, 10, imuCallback);
  corrected_accel_pub_ = nh.advertise<sensor_msgs::Imu>(pub_corrected_accel_topic_name, 10);

  ros::spin();

  return 0;
}