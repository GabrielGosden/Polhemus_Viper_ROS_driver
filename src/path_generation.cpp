/*-------------------------------------------------------------------------------------------------*/
/*																								   */
/* This file is created by Gabriel Gosden. 													   	   */
/* Email: s174865@student.dtu.dk as a part of the bachelor project:								   */
/* "Recreating Operator Paths Using an Electromagnetic Motion Tracker and a Robot Manipulator.	   */
/* This work has been done for Teknologisk Institut, DMRI.										   */
/*																								   */
/*-------------------------------------------------------------------------------------------------*/

#include "ros/ros.h"
#include "std_msgs/String.h"
// Include the custom messages.
#include "viper_tf2_broadcaster/viper_msg_pose_ori.h"
#include "viper_tf2_broadcaster/viper_msg_n.h"
#include "viper_tf2_broadcaster/viper_msg_dist.h"
//Include the path generation functions.
#include "path_generation_functions.h"


/* Global variables*/
double x,y,z,az,el,ro;
int sample_number, sample_time, dist;
double temp;
char input_string[20];

// Callback function for kalman_filter_pose_ori_sub
void kalman_filter_pose_ori_callback(const viper_tf2_broadcaster::viper_msg_pose_ori  &msg)
{
    x = msg.x/100;
    y = msg.y/100;
    z = msg.z/100;
    az = msg.az;
    el = msg.el;
    ro = msg.ro;
	
	z = z*(-1);
	az = az*(-1);
}
// Callback function for viper_broadcaster_n_sub
void viper_broadcaster_n_callback(const viper_tf2_broadcaster::viper_msg_n  &msg)
{
   sample_number = msg.n;
   
}

// Callback function for viper_broadcaster_dist_sub
void viper_broadcaster_dist_callback(const viper_tf2_broadcaster::viper_msg_dist  &msg)
{
   dist = msg.dist;
   
}


double vel;

int main(int argc, char **argv)
{
	// Create ROS node
	ros::init(argc, argv, "path_generation");
	// Create ROS nodehandle
	ros::NodeHandle n;
	// Get filtered position and orientation from the kalman filter.
	ros::Subscriber kalman_filter_pose_ori_sub = n.subscribe("kalman_filter_pose_ori",10,kalman_filter_pose_ori_callback);
	// Get the sample number from the viper.
	ros::Subscriber viper_broadcaster_n_sub = n.subscribe("viper_broadcaster_n",10,viper_broadcaster_n_callback);
	// Get distortion level from the viper
	ros::Subscriber viper_broadcaster_dist_sub = n.subscribe("viper_broadcaster_dist",10,viper_broadcaster_dist_callback);
	// Publisher for movel to UR
	ros::Publisher ur_script_pub = n.advertise<std_msgs::String>("ur_hardware_interface/script_command",10);
	
	
	// Get ROS loop_rate
	n.getParam("/path_sample_time", sample_time);
	ros::Rate loop_rate(sample_time);

	//ROS_INFO("Please ensure Viper is configured and coordinates are correct. \n");
	//ROS_INFO("Please place sensor in start position to ensure safe start. \n");
	//ROS_INFO("When ready write 'run' to start robot.");

    while (ros::ok())
  {
	

	//ROS_INFO("x = %f, y = %f, z = %f",y,x,z);
	// String for servoj command
	std_msgs::String msg;
	std::stringstream ss;
	//temp = x;
	//x = y;
	//y = temp;
	// Fill string with position data and apply rotation matrix (Rot(x,pi)*Rot(z,pi/2))
	//ss << "servoj(get_inverse_kin(p[" << y << "," << x << "," << z*(-1) << ",3.72,-0.3,-0.12]), t=0.08, lookahead_time=0.001, gain=100)";
	//ss << "servoj(get_inverse_kin(p[" << y << "," << x << "," << z << "," << el << "," << ro << "," << az <<"]), t=0.08, lookahead_time=0.001, gain=100)";
	ss << "servoj(get_inverse_kin(p[" << y << "," << x << "," << z << "," << el << "," << ro << "," << az <<"]), t=0.5, lookahead_time=0.001, gain=100)";
	msg.data = ss.str();

	


	// Safety for testing
	ROS_INFO("x = %f, y = %f, z = %f, az = %f, el = %f, ro = %f",y,x,z,az,el,ro);
		if ( y > -0.5 && y < 0.6){ 		// Robot frame x lim
			if( x > 0 && x < 0.9){ 		// Robot frame y lim
				ur_script_pub.publish(msg);
				//ROS_INFO("Position sent!");
			}
	}else{	
		//ROS_INFO("Position NOT sent, out of bounds!");
	}
	
	
		

	ros::spinOnce();

	loop_rate.sleep();

  }

  return 0;
}
