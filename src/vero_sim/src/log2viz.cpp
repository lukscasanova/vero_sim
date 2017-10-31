#include <ros/ros.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/WrenchStamped.h>

tf::TransformListener* listener_ptr;

void toOdometryMessage(std::string parent, std::string child, nav_msgs::Odometry& odom, ros::Time timestamp){

 	tf::StampedTransform transform;
    try{
      listener_ptr->lookupTransform(parent, child, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      return;
      // ros::Duration(1.0).sleep();
    }

    odom.header.stamp = timestamp;
	odom.header.frame_id = parent;
	odom.child_frame_id = child;

	tf::Vector3 origin = transform.getOrigin();
	tf::Quaternion q = transform.getRotation();

	odom.pose.pose.position.x = origin.x();
	odom.pose.pose.position.y = origin.y();
	odom.pose.pose.position.z = origin.z();

	odom.pose.pose.orientation.x = q.x();
	odom.pose.pose.orientation.y = q.y();
	odom.pose.pose.orientation.z = q.z();
	odom.pose.pose.orientation.w = q.w();


}




int main(int argc, char **argv){

	ros::init(argc, argv, "log2viz");
 
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    std::string log_file_path="/home/lucas/Desktop/3_teste.txt";

	std::ifstream logFile;
	logFile.open(log_file_path.c_str());

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
	ros::Publisher point_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("setpoint", 10);
	ros::Publisher force_pub = nh.advertise<geometry_msgs::WrenchStamped>("force", 10);
	ros::Publisher wind_pub = nh.advertise<geometry_msgs::WrenchStamped>("wind", 10);
	// ros::Publisher fl_pub = nh.advertise<nav_msgs::Odometry>("odom_fl", 10);
	// ros::Publisher fr_pub = nh.advertise<nav_msgs::Odometry>("odom_fr", 10);
	// ros::Publisher rl_pub = nh.advertise<nav_msgs::Odometry>("odom_rl", 10);
	// ros::Publisher rr_pub = nh.advertise<nav_msgs::Odometry>("odom_rr", 10);

	// Initialize pose setpoint message
	
	geometry_msgs::PoseWithCovarianceStamped setpoint;

	setpoint.pose.pose.position.x = 0.0;
	setpoint.pose.pose.position.y = 0.0;
	setpoint.pose.pose.position.z = 50.0;
	setpoint.pose.pose.orientation.x = 0.0;
	setpoint.pose.pose.orientation.y = 0.0;
	setpoint.pose.pose.orientation.z = 0.0;
	setpoint.pose.pose.orientation.w = 1.0;
	setpoint.pose.covariance = {
		6.25,0.0,0.0,0.0,0.0,0.0,
		0.0,6.25,0.0,0.0,0.0,0.0,
		0.0,0.0,0.0,0.0,0.0,0.0,
		0.0,0.0,0.0,1000000.0,0.0,0.0,
		0.0,0.0,0.0,0.0,1000000.0,0.0,
		0.0,0.0,0.0,0.0,0.0,100000.0
	};

	geometry_msgs::WrenchStamped force, wind;

	force.wrench.torque.x = 0.0;
	force.wrench.torque.y = 0.0;
	force.wrench.torque.z = 0.0;

	wind.wrench.torque.x = 0.0;
	wind.wrench.torque.y = 0.0;
	wind.wrench.torque.z = 0.0;


	// double time;
	float x, y, z, row, pitch, yaw, wheel_l, wheel_r;
	float fx, fy, fz;
	float last_fx=0, last_fy=0, last_fz=0;
	float last_mx=0, last_my=0, last_mz=0;
	float mx, my, mz;

	float wind_intensity, wind_yaw;
	static tf::TransformBroadcaster br;
	tf::TransformListener listener;
	listener_ptr = &listener;
	tf::Transform transform;
	// tf::Transform steer_right, steer_left;
	// steer_left.setOrigin(tf::Vector3(2.85, -0.79121, 0));
	// steer_right.setOrigin(tf::Vector3(2.85, 0.79121, 0));

	nav_msgs::Odometry odom_msg;

	ros::Time start_time = ros::Time::now();


	ros::Rate r(20.0);

	while(ros::ok() && logFile.good()){

	 	logFile >> x >> y >> z >>
	 	 row >> pitch >> yaw >>
	 	 fx >> fy >> fz >> 
	 	 mx >> my >> mz >>
	 	 wind_intensity >> wind_yaw;

	 	ROS_INFO("%f, %f, %f, %f, %f, %f", x, y, z, row, pitch, yaw );

	 	y=-y;
	 	z=-z;
	 	pitch = -pitch;
	 	yaw = -yaw;
	 	fy = -fy;
	 	fz = -fz;
	 	my = -my;
	 	mz = -mz;

	 	fx = fx*0.01 + last_fx*0.99;
	 	fy = fy*0.01 + last_fy*0.99;
	 	fz = fz*0.01 + last_fz*0.99;

	 	mx = mx*0.1 + last_mx*0.9;
	 	my = my*0.1 + last_my*0.9;
	 	mz = mz*0.1 + last_mz*0.9;

		ros::Time timestamp = ros::Time::now();

		transform.setOrigin( tf::Vector3(x, y, z) );
		tf::Quaternion q;
		q.setRPY(row, pitch, yaw);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, timestamp, "world", "base_link"));

		odom_msg.header.stamp = timestamp;
		odom_msg.header.frame_id = "world";
		odom_msg.child_frame_id = "base_link";

		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = z;

		odom_msg.pose.pose.orientation.x = q.x();
		odom_msg.pose.pose.orientation.y = q.y();
		odom_msg.pose.pose.orientation.z = q.z();
		odom_msg.pose.pose.orientation.w = q.w();

		odom_pub.publish(odom_msg);

		/*** SETPOINT PUBLISHING ***/
		setpoint.header.stamp = timestamp;
		setpoint.header.frame_id = "world";
		point_pub.publish(setpoint);

		/*** FORCE PUBLISHING ***/
		force.header.stamp = timestamp;
		force.header.frame_id = "base_link";
		force.wrench.force.x = fx;
		force.wrench.force.y = fy;
		force.wrench.force.z = fz;

		force_pub.publish(force);		// q.setRPY(0, 0, wheel_l);

		tf::Vector3 wind_vec(-wind_intensity*cos(wind_yaw), wind_intensity*sin(wind_yaw), 0.0);

		wind_vec = transform.inverse()*wind_vec - transform.inverse()*tf::Vector3(0.0, 0.0, 0.0);
		/*** WIND PUBLISHING ***/
		wind.header.stamp = timestamp;
		wind.header.frame_id = "base_link";
		wind.wrench.force.x = wind_vec.getX();
		wind.wrench.force.y = wind_vec.getY();
		wind.wrench.force.z = wind_vec.getZ();

		wind_pub.publish(wind);
		// steer_left.setRotation(q);

		// q.setRPY(0, 0, wheel_r);
		// steer_right.setRotation(q);

		// br.sendTransform(tf::StampedTransform(steer_right, timestamp, "/mkz/base_link", "/mkz/steer_fr"));
		// br.sendTransform(tf::StampedTransform(steer_left, timestamp, "/mkz/base_link", "/mkz/steer_fl"));

		// toOdometryMessage("world", "/mkz/wheel_rl", odom_msg, timestamp);
		// rl_pub.publish(odom_msg);

		// toOdometryMessage("world", "/mkz/wheel_rr", odom_msg, timestamp);
		// rr_pub.publish(odom_msg);

		// toOdometryMessage("world", "/mkz/wheel_fl", odom_msg, timestamp);
		// fl_pub.publish(odom_msg);

		// toOdometryMessage("world", "/mkz/wheel_fr", odom_msg, timestamp);
		// fr_pub.publish(odom_msg);

		// ROS_INFO("%f, %f, %f, %f, %f, %f", x, y, z, row, pitch, yaw );
		// 
		last_fx = fx;
	 	last_fy = fy;
	 	last_fz = fz;
		r.sleep();
	}
	ROS_INFO_STREAM("end");
}