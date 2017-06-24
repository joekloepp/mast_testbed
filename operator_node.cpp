#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <math.h>
#include <ros/console.h>
#include <cmath>

ros::Publisher leader_follower_pub;
std::string leader,follower;

double roll, pitch, yaw;

int main(int argc, char** argv){

  ros::init(argc, argv, "operator" + follower);
  ROS_INFO("Operator");
  ros::NodeHandle node;

  ros::param::get("~leader", leader);
  ros::param::get("~follower", follower);


  leader_follower_pub = node.advertise<geometry_msgs::Pose>("/operator_cmd" +follower, 100);

  tf::TransformListener listener;
  tf::StampedTransform transform;
	geometry_msgs::Pose pose;
	
  ros::Duration(17.0).sleep();

  ROS_INFO("Operator Waking up");

  ros::Rate rate(10.0);

  while (node.ok()){

    ROS_INFO("Updating Operator Node");


    try{
      listener.lookupTransform(leader,follower, ros::Time(0), transform);
			pose.position.x = transform.getOrigin().x();
			pose.position.y = transform.getOrigin().y();
			pose.position.z = transform.getOrigin().z();

			pose.orientation.x = transform.getRotation().x();
			pose.orientation.y = transform.getRotation().y();
			pose.orientation.z = transform.getRotation().z();
			pose.orientation.w = transform.getRotation().w();

/*
	    leader_x = transform.getOrigin().x();
	    leader_y = transform.getOrigin().y();
	    tf::Matrix3x3(transform.getRotation()).getRPY(leader_roll, leader_pitch, leader_yaw);
	    ROS_INFO("leader_x: %f,leader_y: %f,leader_yaw: %f",leader_x,leader_y,leader_yaw);
*/

  		leader_follower_pub.publish(pose);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
      continue;
    }

    rate.sleep();
  }
  return 0;
};

