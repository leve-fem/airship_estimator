#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
std::string sub_real_odom_topic_name
          , pub_real_odom_topic_name;

nav_msgs::Odometry odom_real_enu_;
ros::Publisher odom_real_pub_; //odometry with ideal values publisher
/**
* Linear velocity callback
*/
void odomRealCallback(const nav_msgs::Odometry::ConstPtr& odom){

  odom_real_enu_ = *odom;
  odom_real_enu_.pose.pose.position.x = odom->pose.pose.position.y;
  odom_real_enu_.pose.pose.position.y = odom->pose.pose.position.x;
  odom_real_enu_.pose.pose.position.z = -odom->pose.pose.position.z;

  odom_real_enu_.twist.twist.linear.z = -odom->twist.twist.linear.z;
  odom_real_enu_.twist.twist.linear.y = -odom->twist.twist.linear.y;

  odom_real_enu_.twist.twist.angular.z = -odom->twist.twist.angular.z;
  odom_real_enu_.twist.twist.angular.y = -odom->twist.twist.angular.y;

  odom_real_enu_.header.frame_id = "world";
  odom_real_enu_.child_frame_id = "base_link_real";
  odom_real_pub_.publish(odom_real_enu_);
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
  ros::init(argc, argv, "frame_converter");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nhPrivate("~");
  ros::NodeHandle nh;

  nhPrivate.param<std::string>("subscriber_topic_name/real/odom_topic_name", sub_real_odom_topic_name, "/odom_real");
  nhPrivate.param<std::string>("publisher_topic_name/real/odom_topic_name", pub_real_odom_topic_name, "/odom_real_enu");
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  //Subscribers
  //Sensors subscribers
  ros::Subscriber sub_odom_real_ = nh.subscribe(sub_real_odom_topic_name, 10, odomRealCallback);
  odom_real_pub_ = nh.advertise<nav_msgs::Odometry>(pub_real_odom_topic_name, 10);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}