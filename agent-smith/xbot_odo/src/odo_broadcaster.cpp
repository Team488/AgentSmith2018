#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "xbot_robot_comms/RobotWheelOdomPacket.h"
#include <Eigen>
#include <cmath>
using namespace Eigen;

const float ODO_RUN_RATE = 50.0;

class OdoNode 
{
  private:
    ros::NodeHandle n;
    ros::Publisher odom_pub;
    ros::Subscriber wheel_odometry_sub;
    ros::Subscriber orientation_sub;
    tf::TransformBroadcaster odom_tf_broadcaster;
    ros::Time current_time, last_time;
    //ros::Rate r(50); //50Hz
    double pX, pY, pTh; //m, m, rads (initial pose)
    double oX, oY, oTh; //m, m, rads (odo update to initial pose)
    double velX, velY, velTh; //m/s, m/s, rads/s (instantaneous velocity in global frame)
    double rVelX, rVelY, rVelTh; //m/s, m/s, rads/s (instantaneous velocity in robot frame)
    Matrix3d mRot;
    Vector3d vOffset, vRot, vNewPos;
    double covPX, covPY, covPZ, covPRoll, covPPitch, covPYaw;
    double covTX, covTY, covTZ, covTRoll, covTPitch, covTYaw;
    double covRTX, covRTY, covRTZ, covRTRoll, covRTPitch, covRTYaw;

    void wheelOdometryCallback(const xbot_robot_comms::RobotWheelOdomPacket::ConstPtr& msg){
      /*
      float32 left_drive_delta cm from last send
      float32 right_drive_delta cm from last send
      float32 time_delta seconds
      */
      //NOTE: We currently assume the robot's rotational center is on the robot center - this is incorrect.
      //TODO: Move all this "information about a robot" to a config file
      double dist_between_wheels = 0.6425; //meters between center to center of wheels along the same axle
      double velR = (msg->right_drive_delta/msg->time_delta)/100.0; //Right Wheel Velocity in m/s
      double velL = (msg->left_drive_delta/msg->time_delta)/100.0; //Left Wheel Velocity in m/s
      if (velR == velL) {
        //Handle the "perfect match" case to avoid the singularity about R.
        velTh = 0.0;
        velX = velR*std::cos(oTh);
        velY = velR*std::sin(oTh);
        oX = oX+velX;
        oY = oY+velY;
      } else {
        //All other cases
        //Based on equations from: https://chess.eecs.berkeley.edu/eecs149/documentation/differentialDrive.pdf
        //R is the distance from the instantaneous center of curvature to the midpoint between the wheels
        //w is the rotational velocity
        //ICC is the instantaneous center of curvature
        double R = (dist_between_wheels/2.0)*((velR + velL)/(velR - velL));
        double w = (velR-velL)/dist_between_wheels;
        double ICCx = oX - R*std::sin(oTh);
        double ICCy = oY + R*std::cos(oTh);
        double wdt = w*msg->time_delta;
        mRot << std::cos(wdt), -std::sin(wdt), 0,
             std::sin(wdt), std::cos(wdt), 0,
             0, 0, 1;
        vOffset << oX - ICCx, oY - ICCy, oTh;
        vRot  << ICCx, ICCy, wdt;
        vNewPos = mRot*vOffset + vRot;
        velX = (vNewPos(0) - oX)/msg->time_delta;
        velY = (vNewPos(1) - oY)/msg->time_delta;
        velTh = (vNewPos(2) - oTh)/msg->time_delta;
        oX = vNewPos(0);
        oY = vNewPos(1);
        oTh = vNewPos(2);
      }
      //Calculate robot frame velocity
      rVelX = (velR+velL)/2;
      rVelY = 0.0;
      rVelTh = (velR-velL)/dist_between_wheels; 
    }

    // void orientationCallback(const geometry_msgs::Quaternion::ConstPtr& msg){
    //   //Do nothing for now
    // }

    // void headingCallback(const std_msgs::Float32::ConstPtr& msg) {
    //   //msg is in degrees!
    //   //TODO: Message needs time delta

    // }
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
    
    /**
     * Update the initial pose without hammering the odo information
     */
    void setInitialPose(double x, double y, double th) {
      pX = x;
      pY = y;
      pTh = th;
    }

    /**
     * Update the current pose of the robot (and hammer the odo information)
     */
    void setCurrentPose(double x, double y, double th) {
      oX = oY = oTh = 0.0;
      pX = x;
      pY = y;
      pTh = th;
    }

    

    void subPubSetup() {
      odom_pub = n.advertise<nav_msgs::Odometry>("rawsensor/wheel_odom", 50);
      wheel_odometry_sub = n.subscribe("/comms/wheel_odometry", 5, &OdoNode::wheelOdometryCallback, this);
      
      // ros::Subscriber orientation_sub = n.subscribe("/comms/orientation", 5, &OdoNode::orientationCallback, this);
      // ros::Subscriber heading_sub = n.subscribe("/comms/heading", 5, &OdoNode::headingCallbcak, this);
      current_time = ros::Time::now();
      last_time = ros::Time::now();
      //r = new ros::Rate(50.0); //Run at 50 Hz.
      //Set the initial pose
      pX = pY = pTh = 0.0;
      oX = oY = oTh = 0.0;
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

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(oTh);

        // publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "rawsensor/wheel_odom";

        //set the position
        odom.pose.pose.position.x = oX;
        odom.pose.pose.position.y = oY;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //The following covariance matrix is total BS.
        odom.pose.covariance = getPoseCovariance();

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
