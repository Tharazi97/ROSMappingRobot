#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "mapping_robot/GetEncoder.h"
#include "mapping_robot/Speeds.h"
#include "mapping_robot/Encoder.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher speed_pub = n.advertise<mapping_robot::Speeds>("speeds", 10);
  tf::TransformBroadcaster odom_broadcaster;

  ros::ServiceClient clientLeft = n.serviceClient<mapping_robot::GetEncoder>("GetEncoderL");
  ros::ServiceClient clientRight = n.serviceClient<mapping_robot::GetEncoder>("GetEncoderR");

  double xP = 0.0;
  double yP = 0.0;
  double theta = 0.0;

  double dxP = 0.0;
  double dyP = 0.0;
  double dtheta = 0.0;
  
  double dFiL = 0.0;
  double dFiR = 0.0;

  double vp = 0.0;

  double wheelR = 0.0345;
  double shaft = 0.173;
 // uint8_t counter = 9;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);
  while(n.ok()) {

    ros::spinOnce();               // check for incoming messages
   // if (counter >= 9) {
      current_time = ros::Time::now();
      double dt = (current_time - last_time).toSec();

      mapping_robot::GetEncoder srvLeft;
      if (clientLeft.call(srvLeft)) {
        double difference = current_time.toSec() - srvLeft.response.data.timeStamp;
        if (difference <= dt) {
          if (srvLeft.response.data.delta != 0) {
            dFiL = (3.14/20)/srvLeft.response.data.delta;
          }
        }/* else if (difference <= 2*dt) {
          if (dFiL > 0) {
            dFiL = (3.14/20)/dt;
          } else {
            dFiL = (-3.14/20)/dt;
          }
        } else if (difference <= 3*dt) {
          if (dFiL > 0) {
            dFiL = (3.14/20)/(2*dt);
          } else {
            dFiL = (-3.14/20)/(2*dt);
          }
        }*/ else {
          dFiL = 0;
        }
      } else {
        ROS_INFO("fail");
        dFiL = 0;
      }

      mapping_robot::GetEncoder srvRight;
      if (clientRight.call(srvRight)) {
        double difference = current_time.toSec() - srvRight.response.data.timeStamp;
        if (difference <= dt) {
          if (srvRight.response.data.delta != 0) {
            dFiR = (3.14/20)/srvRight.response.data.delta;
          }
        }/* else if (difference <= 2*dt) {
          if (dFiR > 0) {
            dFiR = (3.14/20)/dt;
          } else {
            dFiR = (-3.14/20)/dt;
          }
        } else if (difference <= 3*dt) {
          if (dFiR > 0) {
            dFiR = (3.14/20)/(2*dt);
          } else {
            dFiR = (-3.14/20)/(2*dt);
          }
        }*/ else {
          dFiR = 0;
        }
      } else {
        ROS_INFO("fail");
        dFiR = 0;
      }
      
      mapping_robot::Speeds speeds;
      speeds.left = dFiL;
      speeds.right = dFiR;
      speed_pub.publish(speeds);
      

      vp = wheelR*(dFiR + dFiL)/2;
      dtheta = wheelR*(dFiR - dFiL)/shaft;

      dxP = cos(theta) * vp;
      dyP = sin(theta) * vp;

      //compute odometry in a typical way given the velocities of the robot
      double delta_x = dxP * dt;
      double delta_y = dyP * dt;
      double delta_th = dtheta * dt;

      xP += delta_x;
      yP += delta_y;
      theta += delta_th;

      //since all odometry is 6DOF we'll need a quaternion created from yaw
      tf::Quaternion quat = tf::createQuaternionFromYaw(theta);
      quat.normalize();
      geometry_msgs::Quaternion odom_quat;
      tf::quaternionTFToMsg(quat, odom_quat);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = xP;
      odom_trans.transform.translation.y = yP;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);
      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = xP;
      odom.pose.pose.position.y = yP;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = dxP;
      odom.twist.twist.linear.y = dyP;
      odom.twist.twist.angular.z = dtheta;

      //publish the message
      odom_pub.publish(odom);

      //counter = 0;
      last_time = current_time;
    /*} else {
      //since all odometry is 6DOF we'll need a quaternion created from yaw
      tf::Quaternion quat = tf::createQuaternionFromYaw(theta);
      quat.normalize();
      geometry_msgs::Quaternion odom_quat;
      tf::quaternionTFToMsg(quat, odom_quat);

      //first, we'll publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = xP;
      odom_trans.transform.translation.y = yP;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;

      //send the transform
      odom_broadcaster.sendTransform(odom_trans);
      //next, we'll publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      //set the position
      odom.pose.pose.position.x = xP;
      odom.pose.pose.position.y = yP;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;

      //set the velocity
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = dxP;
      odom.twist.twist.linear.y = dyP;
      odom.twist.twist.angular.z = dtheta;

      //publish the message
      odom_pub.publish(odom);

      ++counter;
    }*/
    r.sleep();
  }
}
