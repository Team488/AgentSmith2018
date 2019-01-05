#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include <Eigen>
#include <cmath>
using namespace Eigen;

const float ODO_RUN_RATE = 50.0;

class OdoNode 
{
  private:
    ros::NodeHandle n;
    ros::Publisher odom_pub;
    ros::Subscriber wheel_velocity_sub;
    ros::Subscriber orientation_sub;
    tf::TransformBroadcaster odom_tf_broadcaster;
    ros::Time current_time, last_time;
    //ros::Rate r(50); //50Hz
    double rVelX, rVelY, rVelTh; //m/s, m/s, rads/s (instantaneous velocity in robot frame)
    Matrix3d mRot;
    Vector3d vOffset, vRot, vNewPos;
    double covPX, covPY, covPZ, covPRoll, covPPitch, covPYaw;
    double covTX, covTY, covTZ, covTRoll, covTPitch, covTYaw;
    double covRTX, covRTY, covRTZ, covRTRoll, covRTPitch, covRTYaw;

    void wheelVelocityCallback(const nav_msgs::Odometry::ConstPtr& msg){
      rVelX = std::sqrt(msg->twist.twist.linear.x * msg->twist.twist.linear.x + msg->twist.twist.linear.y * msg->twist.twist.linear.y);
      rVelY = 0.0;
      rVelTh = 0.0; //THIS IS WRONG, but hacks a-go!
    }

    boost::array<double, 36> getPoseCovariance() {
      boost::array<double, 36> m = 
               {covPX, 0, 0, 0, 0, 0,
               0, covPY, 0, 0, 0, 0,
               0, 0, covPZ, 0, 0, 0,
               0, 0, 0, covPRoll, 0, 0,
               0, 0, 0, 0, covPPitch, 0,
               0, 0, 0, 0, 0, covPYaw};
      return m;
    }

    boost::array<double, 36> getTwistCovariance() {
      boost::array<double, 36> m = 
               {covTX, 0, 0, 0, 0, 0,
               0, covTY, 0, 0, 0, 0,
               0, 0, covTZ, 0, 0, 0,
               0, 0, 0, covTRoll, 0, 0,
               0, 0, 0, 0, covTPitch, 0,
               0, 0, 0, 0, 0, covTYaw};
      return m;
    }

    boost::array<double, 36> getRobotTwistCovariance() {
      boost::array<double, 36> m = 
               {covRTX, 0, 0, 0, 0, 0,
               0, covRTY, 0, 0, 0, 0,
               0, 0, covRTZ, 0, 0, 0,
               0, 0, 0, covRTRoll, 0, 0,
               0, 0, 0, 0, covRTPitch, 0,
               0, 0, 0, 0, 0, covRTYaw};
      return m;
    }

  public:
 

    void subPubSetup() {
      odom_pub = n.advertise<nav_msgs::Odometry>("rawsensor/wheel_odom", 50);
      wheel_velocity_sub = n.subscribe("/sim/main_odom/data", 5, &OdoNode::wheelVelocityCallback, this);
      
      // ros::Subscriber orientation_sub = n.subscribe("/comms/orientation", 5, &OdoNode::orientationCallback, this);
      // ros::Subscriber heading_sub = n.subscribe("/comms/heading", 5, &OdoNode::headingCallbcak, this);
      current_time = ros::Time::now();
      last_time = ros::Time::now();
      //r = new ros::Rate(50.0); //Run at 50 Hz.
      //Set the initial covariances
      covPX = 0.01;
      covPY = 0.01;
      covPZ = 0.01;
      covPRoll = 0.1;
      covPPitch = 0.1;
      covPYaw = 0.1;
      covTX = 0.01;
      covTY = 0.01;
      covTZ = 0.01;
      covTRoll = 0.1;
      covTPitch = 0.1;
      covTYaw = 0.1;
      covRTX = 0.01;
      covRTY = 0.01;
      covRTZ = 0.01;
      covRTRoll = 0.1;
      covRTPitch = 0.1;
      covRTYaw = 0.1;
    }

    void run() {
      //Do all the work here.
      ros::Rate r(ODO_RUN_RATE);
      while(n.ok()) {
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        // publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "rawsensor/wheel_odom";

        // //set the position
        // odom.pose.pose.position.x = oX;
        // odom.pose.pose.position.y = oY;
        // odom.pose.pose.position.z = 0.0;
        // odom.pose.pose.orientation = odom_quat;
        // //The following covariance matrix is total BS.
        // odom.pose.covariance = getPoseCovariance();

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = rVelX;
        odom.twist.twist.linear.y = rVelY;
        odom.twist.twist.angular.z = rVelTh;
        //The following covariance matrix is total BS - a little better now.
        odom.twist.covariance = getRobotTwistCovariance();

        //publish the message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();

      }
    }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  OdoNode node;
  node.subPubSetup();
  node.run();

}
