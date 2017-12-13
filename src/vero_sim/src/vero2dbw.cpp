#include <ros/ros.h>
#include <ransac_corridor_control/CarCommand.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>
#include <geometry_msgs/Twist.h>

ros::Publisher dbw_throttle_pub, dbw_brake_pub, dbw_steer_pub;
ros::Publisher cmd_vel_pub;

void vero_cmd_callback(ransac_corridor_control::CarCommand cmd){
	double steer_angle = cmd.steerAngle;
	double speed_left = cmd.speedLeft;
	double speed_right = cmd.speedRight;

	// // Steering Angle
	// dbw_mkz_msgs::SteeringCmd steer_msg;
	// steer_msg.steering_wheel_angle_cmd = steer_angle*14.0; //numero mistico
	// steer_msg.enable = true;
	// dbw_steer_pub.publish(steer_msg);

	// // Throttle
	// dbw_mkz_msgs::ThrottleCmd throttle_msg;
	// throttle_msg.pedal_cmd = 0.01*(speed_left+speed_right)/2.0;
	// throttle_msg.enable = true;
	// dbw_throttle_pub.publish(throttle_msg);
	geometry_msgs::Twist cmd_vel;

	cmd_vel.linear.x = (speed_left+speed_right)/2.0;
	cmd_vel.angular.z = tan(steer_angle)*cmd_vel.linear.x/2.8498; // L = 2.8498

	cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "vero2dbw");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    std::string vero_cmd_topic = "/verocarfreedom/car_command";
    std::string throttle_cmd_topic = "/mkz/throttle_cmd";
    std::string brake_cmd_topic = "/mkz/brake_cmd";
    std::string steer_cmd_topic = "/mkz/steering_cmd";
    std::string cmd_vel_topic = "/mkz/cmd_vel";

    // Publish to DBW commands
    dbw_throttle_pub = nh.advertise<dbw_mkz_msgs::ThrottleCmd>(throttle_cmd_topic, 1);
    dbw_brake_pub = nh.advertise<dbw_mkz_msgs::BrakeCmd>(brake_cmd_topic, 1);
    dbw_steer_pub = nh.advertise<dbw_mkz_msgs::SteeringCmd>(steer_cmd_topic, 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

    ros::spin();
}
