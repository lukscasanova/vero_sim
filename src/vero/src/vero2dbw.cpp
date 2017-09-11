#include <ros/ros.h>
#include <ransac_project/CarCommand.h>
#include <dbw_mkz_msgs/ThrottleCmd.h>
#include <dbw_mkz_msgs/BrakeCmd.h>
#include <dbw_mkz_msgs/SteeringCmd.h>


ros::Publisher dbw_throttle_pub, dbw_brake_pub, dbw_steer_pub;

void vero_cmd_callback(ransac_project::CarCommand cmd){
	double steer_angle = cmd.steerAngle;
	double speed_left = cmd.speedLeft;
	double speed_right = cmd.speedRight;

	// Steering Angle
	dbw_mkz_msgs::SteeringCmd steer_msg;
	steer_msg.steering_wheel_angle_cmd = steer_angle;
	steer_msg.enable = true;
	dbw_steer_pub.publish(steer_msg);

	// Throttle 
	dbw_mkz_msgs::ThrottleCmd throttle_msg;
	throttle_msg.pedal_cmd = (speed_left+speed_right)/2.0;
	throttle_msg.enable = true;
	dbw_throttle_pub.publish(throttle_msg);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "vero2dbw");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    std::string vero_cmd_topic = "/verocarfreedom/car_command";
    std::string throttle_cmd_topic = "/mkz/throttle_cmd";
    std::string brake_cmd_topic = "/mkz/brake_cmd";
    std::string steer_cmd_topic = "/mkz/steer_cmd";
    
    //Subscribe to Vero command
    ros::Subscriber vero_cmd_sub = nh.subscribe(vero_cmd_topic, 1, &vero_cmd_callback);

    // Publish to DBW commands
    dbw_throttle_pub = nh.advertise<dbw_mkz_msgs::ThrottleCmd>(throttle_cmd_topic, 1);
    dbw_brake_pub = nh.advertise<dbw_mkz_msgs::BrakeCmd>(brake_cmd_topic, 1);
    dbw_steer_pub = nh.advertise<dbw_mkz_msgs::SteeringCmd>(steer_cmd_topic, 1);

    ros::spin();
}