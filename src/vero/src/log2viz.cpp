#include <ros/ros.h>
#include <fstream>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


int main(int argc, char **argv){

	ros::init(argc, argv, "log2viz");
 
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    std::string log_file_path="/home/lucas/Desktop/driftdata.txt";

	std::ifstream logFile;
	logFile.open(log_file_path.c_str());

	ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);



	double time;
	float x, y, z, row, pitch, yaw, wheel_l, wheel_r;
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Transform steer_right, steer_left;
	steer_left.setOrigin(tf::Vector3(2.85, -0.79121, 0));
	steer_right.setOrigin(tf::Vector3(2.85, 0.79121, 0));

	nav_msgs::Odometry odom_msg;


	// ros::Rate r(100.0);

	while(ros::ok() && logFile.good()){
	 	logFile >> time >> x >> y >> z >> row >> pitch >> yaw >> wheel_l >> wheel_r;

		transform.setOrigin( tf::Vector3(x, y, z) );
		tf::Quaternion q;
		q.setRPY(row, pitch, yaw);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/mkz/base_footprint"));

		odom_msg.header.stamp = ros::Time::now();
		odom_msg.header.frame_id = "world";
		odom_msg.child_frame_id = "/mkz/base_link";

		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = z;

		odom_msg.pose.pose.orientation.x = q.x();
		odom_msg.pose.pose.orientation.y = q.y();
		odom_msg.pose.pose.orientation.z = q.z();
		odom_msg.pose.pose.orientation.w = q.w();

		odom_pub.publish(odom_msg);
		q.setRPY(0, 0, wheel_l);
		steer_left.setRotation(q);

		q.setRPY(0, 0, wheel_r);
		steer_right.setRotation(q);

		br.sendTransform(tf::StampedTransform(steer_right, ros::Time::now(), "/mkz/base_link", "/mkz/steer_fr"));
		br.sendTransform(tf::StampedTransform(steer_left, ros::Time::now(), "/mkz/base_link", "/mkz/steer_fl"));




		ROS_INFO_STREAM(time << x << y << z << row << pitch << yaw << wheel_l << wheel_r);
		// r.sleep();
	}
}