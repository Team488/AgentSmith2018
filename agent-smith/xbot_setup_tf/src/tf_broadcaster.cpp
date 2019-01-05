#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <cmath>

#define PI (3.14159265)

int main(int argc, char** argv){
	ros::init(argc, argv, "robot_tf_publisher");
	ros::NodeHandle n;
	ros::Rate r(100);

	tf::TransformBroadcaster broadcaster;

	while(n.ok()){
		broadcaster.sendTransform(
			tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-0.4125, -0.3495, 0.0)),
			ros::Time::now(), "base_link", "robot_frame"));

		broadcaster.sendTransform(
			tf::StampedTransform(
			// Practice bot:
			//tf::Transform(tf::createQuaternionFromYaw(PI), tf::Vector3(0.5715 - 0.0381, 0.1651 - 0.117, 0.254 + 0.038)),
			tf::Transform(tf::createQuaternionFromYaw(PI), tf::Vector3(0.575-0.010, -0.45 - 0.165, 0.11)),
			ros::Time::now(),"robot_frame", "base_laser"));
		
		broadcaster.sendTransform(
			tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, -std::sqrt(0.5), std::sqrt(0.5)), tf::Vector3(0.355, 0.5492, 0.058)),
			ros::Time::now(),"robot_frame", "navx_frame"));
		
		broadcaster.sendTransform(
			tf::StampedTransform(
			// Practice bot: 
			//tf::Transform(tf::createQuaternionFromRPY(0, 23 * PI / 180, 0), tf::Vector3(0.5715, 0.1615 - 0.037, 0.875 - 0.395 + 0.254)),
			tf::Transform(tf::createQuaternionFromRPY(0, 23 * PI / 180, 0), tf::Vector3(0.5715, 0.1615 - 0.037, 0.813)),
			ros::Time::now(),"robot_frame", "base_zed"));
		
		broadcaster.sendTransform(
			tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0, -0.133)),
			ros::Time::now(),"base_link", "gripper_middle"));

		r.sleep();
	}
}
