#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <vector> // for vector 
#include <iterator> // for back_inserter 
#include <Eigen/Geometry>

#define NUM_STATES 7
#define NUM_MEASURES 4

nav_msgs::Odometry odom;
geometry_msgs::AccelStamped accel;
double pitotPressure;
ros::Publisher estimated_wind_pub_; //wind speed publisher
std::string pub_estimated_wind_topic_name //topic name
          , sub_odom_topic_name
          , sub_accel_topic_name
          , sensor_frame_id
          , robot_frame_id
          , sub_pitot_topic_name
          ;

Eigen::MatrixXd Q(NUM_STATES,NUM_STATES)//Process covariance
			  , R(NUM_MEASURES,NUM_MEASURES)//Measure covariance
			  , F(NUM_STATES,NUM_STATES)//Model
			  , H(NUM_MEASURES,NUM_STATES)//Observation Matrix
			  , P(NUM_STATES,NUM_STATES)
			  , x_hat(NUM_STATES,1)
			  , y_hat(NUM_MEASURES,1)
			  , zk(NUM_MEASURES,1)
			  , C(NUM_MEASURES, NUM_MEASURES)
			  , K(NUM_STATES, NUM_MEASURES)
			  , I(NUM_STATES, NUM_STATES)
			  , P_0(NUM_STATES,NUM_STATES)
			  , S(3,3)
			  , h_xk(4,1)
			  , L(NUM_STATES,NUM_STATES)
			  ;

double lambda_rdot;

void predict();

void update();

void loadParams(ros::NodeHandle nh);

void updateRotationMatrix(geometry_msgs::Quaternion orientation);

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	odom = *odom_msg;

	predict();

	update();
}

void accelCallback(const geometry_msgs::AccelStamped::ConstPtr& accel_msg){
	accel = *accel_msg;
}

void pitotCallback(const std_msgs::Float64& pitot_msg){
	pitotPressure = pitot_msg.data;//0.57*pow(pitot_msg.data,2);

	//Update pitot covariance
	double r_dot = std::abs(accel.accel.angular.z);
	// for(int i=0;i<NUM_MEASURES; i++)
		R(3,3) = R(3,3)*(1.0+lambda_rdot*r_dot);

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
	ros::init(argc, argv, "wind_estimator");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle nhPrivate("~");
	ros::NodeHandle nh;
	loadParams(nhPrivate);

	ros::Subscriber sub_odom_ = nh.subscribe(sub_odom_topic_name, 10, odomCallback);
	ros::Subscriber sub_accel_filtered_ = nh.subscribe(sub_accel_topic_name, 10, accelCallback);
	ros::Subscriber sub_pitot_ = nh.subscribe(sub_pitot_topic_name, 10, pitotCallback);

	estimated_wind_pub_ = nh.advertise<geometry_msgs::Vector3Stamped>(pub_estimated_wind_topic_name, 10);


	ros::spin();

	return 0;
}	

void loadParams(ros::NodeHandle nh){

	nh.param<std::string>("pub_estimated_wind_topic_name", pub_estimated_wind_topic_name, "/wind_speed/filtered");
	nh.param<std::string>("sub_odom_topic_name", sub_odom_topic_name, "/odometry/filtered");
	nh.param<std::string>("sub_accel_topic_name", sub_accel_topic_name, "/accel/filtered");
	nh.param<std::string>("sub_pitot_topic_name", sub_pitot_topic_name, "/droni_sensors/pitot_pressure");

	nh.param<std::string>("robot_frame_id", robot_frame_id, "base_link");
	nh.param<std::string>("sensor_frame_id", sensor_frame_id, "gondola_link");
	nh.param<double>("lambda_rdot", lambda_rdot, 1000.0);

	std::vector<double> process_covariance(NUM_STATES, 1.0);
	if(nh.hasParam("process_covariance"))
		nh.getParam("process_covariance", process_covariance);	
	for(int i=0; i<NUM_STATES;i++){
		Q(i,i) = process_covariance.at(i);
	}

	std::vector<double> measurement_covariance(NUM_MEASURES, 1.0);
	if(nh.hasParam("measurement_covariance"))
		nh.getParam("measurement_covariance", measurement_covariance);
	for(int i=0; i<NUM_MEASURES;i++){
		R(i,i) = measurement_covariance.at(i);
	}

	std::vector<double> initial_covariance(NUM_STATES, 1.0);
	if(nh.hasParam("initial_covariance"))
		nh.getParam("initial_covariance", initial_covariance);
	for(int i=0; i<NUM_STATES;i++){
		P(i,i) = initial_covariance.at(i);
	}

	F.setIdentity(NUM_STATES, NUM_STATES);

	S.setIdentity(3, 3);

	std::vector<double> initial_state(NUM_STATES, 1.0);
	if(nh.hasParam("initial_state"))
		nh.getParam("initial_state", initial_state);
	for(int i=0; i<NUM_STATES;i++){
		x_hat(i,0) = initial_state.at(i);
	}

	I.setIdentity(NUM_STATES, NUM_STATES);
	std::cout << "I: " << I << std::endl;
	std::cout << "R: " << R << std::endl;
	std::cout << "Q: " << Q << std::endl;

    pitotPressure=0.0;
}
void predict(){

	//Updating matrix S
	updateRotationMatrix(odom.pose.pose.orientation);
	
	Eigen::MatrixXd Vg(3,1), Vg_enu(3,1);
	Vg(0,0) = odom.twist.twist.linear.x;
	Vg(1,0) = odom.twist.twist.linear.y;
	Vg(2,0) = odom.twist.twist.linear.z;
	Vg_enu = S*Vg;

	zk(0,0) = Vg_enu(0,0);
	zk(1,0) = Vg_enu(1,0);
	zk(2,0) = Vg_enu(2,0);
	zk(3,0) = pitotPressure;
	P = F*P*F.transpose() + Q;
}

void update(){
	Eigen::MatrixXd Vw_hat(3,1), Va_hat(3,1);

	
	double ua_hat = x_hat(4,0);
	Vw_hat << x_hat(0,0),
			  x_hat(1,0),
			  x_hat(2,0);
  	double eta = x_hat(3,0);
	Va_hat << x_hat(4,0),
			  x_hat(5,0),
			  x_hat(6,0);

	h_xk << Vw_hat + S*Va_hat,
			eta*pow(ua_hat,2);

	H << 1, 0, 0, 0, S(0,0), S(0,1), S(0,2),
		 0, 1, 0, 0, S(1,0), S(1,1), S(1,2),
		 0, 0, 1, 0, S(2,0), S(2,1), S(2,2),
		 0, 0, 0, ua_hat*ua_hat, 2*eta*ua_hat, 0, 0;

	C = H*P*H.transpose() + R;

	K = P*H.transpose()*C.inverse();
	x_hat = x_hat+K*(zk - h_xk);
	L = (I - K*H);
	P = L*P; 

	geometry_msgs::Vector3Stamped windSpeed;
	windSpeed.vector.x=x_hat(0,0);
	windSpeed.vector.y=x_hat(1,0);
	windSpeed.vector.z=x_hat(2,0);
	windSpeed.header.stamp = ros::Time::now();
	windSpeed.header.frame_id = robot_frame_id;

	// ROS_INFO_STREAM("x_hat: " << x_hat);
	// ROS_INFO("wind_x:%f", x_hat(0,0));
	// ROS_INFO("wind_y:%f", x_hat(1,0));
	// ROS_INFO("wind_z:%f", x_hat(2,0));
	// ROS_INFO("eta: %f", eta);

	estimated_wind_pub_.publish(windSpeed);
}

void updateRotationMatrix(geometry_msgs::Quaternion orientation){
	tf::Quaternion tf_q;
	tf::quaternionMsgToTF(orientation, tf_q);

	double roll, pitch, yaw;
	tf::Matrix3x3(tf_q).getRPY( roll, pitch,yaw);
	// std::cout << "tf rpy: " << roll << "," << pitch << "," << yaw << std::endl;

	tf::Quaternion tf_q_enu = tf::createQuaternionFromRPY(roll,  pitch, yaw);

	Eigen::Quaterniond q(tf_q_enu.getW(), tf_q_enu.getX(), tf_q_enu.getY(), tf_q_enu.getZ());

	S = q.normalized().toRotationMatrix();
	// Eigen::MatrixXd x_unit(3,1);
	// Eigen::MatrixXd x_enu(3,1);
	// x_unit(0,0) = 1.0;
	// x_unit(1,0) = 0.0;
	// x_unit(2,0) = 0.0;

	// x_enu = S*x_unit;

	// std::cout << "S: " << S(0,0) << "," << S(1,0) << "," << S(2,0) << std::endl;
	// std::cout << "x_enu: " << x_enu << std::endl;
}
