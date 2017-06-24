#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <math.h>
#include <ros/console.h>
#include <cmath>

ros::Publisher cmd_vel_pub;
ros::Subscriber sub;

std::string robot_name,follower;

float x,y;
float e_l,e_y;
float kp_l = 0.3;
float kp_y = 0.7;
float x_d = 0;
float y_d = -0.2;
float yaw_d = 0; 
int n = 0;
int no_of_waypoints = 8;

float waypoints_x[] = {0,  0,   0,   0,   0,  0,     0, 0};
float waypoints_y[] = {-0.2   ,  -0.2,  -0.2,   -0.2,  -0.2,  -0.2, -0.2, -0.2 };


double roll, pitch, yaw;

void chatterCallback(const geometry_msgs::Pose& msg);

int main(int argc, char** argv){


  ros::init(argc, argv, "operator" + follower);
  ROS_INFO("Publisher");
  ros::NodeHandle node;

  ros::param::get("~follower", follower);
  ros::param::get("~robot_name", robot_name);



  cmd_vel_pub = node.advertise<geometry_msgs::Twist>("/"+robot_name+"/cmd_vel", 100);
  sub = node.subscribe("/operator_cmd"+follower, 1000, chatterCallback);

  ROS_INFO("Follower Waking up");
  ros::spin(); // Run until interupted 
};


void chatterCallback(const geometry_msgs::Pose& msg)
{
   ROS_INFO("Updating Leader Node");

    geometry_msgs::Twist base_cmd;

	    x = msg.position.x;
	    y = msg.position.y;

	    tf::Quaternion q(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);

	    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	    ROS_INFO("x: %f,y: %f,yaw: %f",x,y,yaw);


	    yaw_d = -1*atan2((x_d-x) , (y_d-y));
	    //Controller
	    e_l = sqrt((x_d - x)*(x_d - x)+(y_d-y)*(y_d-y));

	  if ( (yaw_d-yaw) < -3*3.14/2 || (yaw_d-yaw) > 3*3.14/2 )
	    e_y = 0.2*(yaw_d-yaw);
	  else
	    e_y = -1*(yaw_d-yaw);


	    ROS_INFO("e_l: %f",e_l);
	    ROS_INFO("e_yaw: %f",e_y);


	    //check if Way-point reached
	    ROS_INFO("x_d: %f,y_d: %f",x_d,y_d);
	    ROS_INFO("yaw_d: %f",yaw_d);


	
	    if(e_l < 0.025 && e_l > -0.025){
		    ROS_INFO("abs(e_l) < 0.01");




		    ROS_INFO("Desired positions updated.");
	        ROS_INFO("x_d: %f,y_d: %f",x_d,y_d);
	        ROS_INFO("yaw_d: %f",yaw_d);
		
		    base_cmd.linear.x = 0.00;
		    base_cmd.angular.z = 0.00;

 		    n = n+1;
		    if(n == no_of_waypoints){
		        n = 0;
		    }


		    x_d = waypoints_x[n];
		    y_d = waypoints_y[n];	
	    }
	   else {
	      //we will be sending commands of type "twist"
	      base_cmd.linear.x = kp_l*e_l;
	      base_cmd.angular.z = kp_y*e_y;
	    }

	    ROS_INFO("Linear Cmd: %f",base_cmd.linear.x);
	    ROS_INFO("Angular Cmd: %f",base_cmd.angular.z);

	    cmd_vel_pub.publish(base_cmd);
	    ROS_INFO("n: %i",n);
  

}
