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
int sample_number, sample_time, dist,state = 1, init_run = 0;
char input_string[20];

// Test variables. Delete later
double temp;


ros::Publisher ur_script_pub;


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
	//ros::Publisher ur_script_pub = n.advertise<std_msgs::String>("ur_hardware_interface/script_command",10);
	
	// Get ROS loop_rate
	n.getParam("/path_sample_time", sample_time);
	ros::Rate loop_rate(sample_time);


    while (ros::ok())
  {
	// String for servoj command
	std_msgs::String msg;
	std::stringstream ss;


	// Fill string with position data and apply rotation matrix (Rot(x,pi)*Rot(z,pi/2))
	//ss << "servoj(get_inverse_kin(p[" << y << "," << x << "," << z << "," << el << "," << ro << "," << az <<"]), t=0.08, lookahead_time=0.001, gain=100)";
	//ss << "servoj(get_inverse_kin(p[" << y << "," << x << "," << z << "," << el << "," << ro << "," << az <<"]), t=0.5, lookahead_time=0.001, gain=100)";
	
	// Put generated string into message
	//msg.data = ss.str();
	//ss << "movej([-1.571,-1.571,1.571,-0.576,1.571,0],a=0.1,v=0.1)";
	//msg.data = ss.str();
	//ur_script_pub.publish(msg);
	
	switch (state){
		//Move robot to safe starting position
		case 1:
			ur_script_pub = n.advertise<std_msgs::String>("ur_hardware_interface/script_command",10);
			ros::Duration(0.5).sleep();
			ss << "movej([-1.571,-1.571,1.571,-0.576,1.571,0],a=0.1,v=0.1)";
			msg.data = ss.str();
			ur_script_pub.publish(msg);
			ROS_INFO("Sending robot to starting position, please wait.");
			state=2;

		// Confirm by user that robot is at safe starting position.
		case 2:
			ROS_INFO("Confirm robot is at starting position by pressing 'Enter'.");
			fgets(input_string,100,stdin);
			state=3;
		break;
		
		// Move sensor to safe starting position.
		case 3: 
			ROS_INFO("Please move Viper sensor to the safe starting position and orientation");
			if(x > 0.5 && x < 0.55){
				if(y > -0.40 && y < 0.40){
					if(z > -0.40 && z < 0.40){
							ROS_INFO("Sensor is in safe starting position.");
							state = 4;
					}else{
						ROS_INFO("Sensor Z limit.");
					}
				}else{
					ROS_INFO("Sensor X limit.");
				}
			}else{
				ROS_INFO("Sensor Y limit.");
			}
		break;
		// Confirm by user to start robot movement.
		case 4:
			ROS_INFO("To start robot movement press 'Enter'");
			fgets(input_string,100,stdin);
			state = 5;
		break;

		// Running the robot.
		case 5:
		if (init_run = 0){
			ur_script_pub = n.advertise<std_msgs::String>("ur_hardware_interface/script_command",10);
			ros::Duration(0.5).sleep();
			init_run = 1;
		}
		
		if(x > 0 && x < 1.00){
			if(y > -0.40 && y < 0.40){
				if(z > -0.40 && z < 0.40){
						ROS_INFO("x = %f, y = %f, z = %f, az = %f, el = %f, ro = %f",y,x,z,az,el,ro);
						ss << "servoj(get_inverse_kin(p[" << y << "," << x << "," << z << "," << el << "," << ro << "," << az <<"]), t=0.08, lookahead_time=0.001, gain=100)";
						msg.data = ss.str();
						ur_script_pub.publish(msg);
						//ROS_INFO("Within limits, sending command");
				}else{
					ROS_WARN("Robot stopped: Z limit.");
				}
			}else{
				ROS_WARN("Robot stopped: X limit.");
			}
		}else{
			ROS_WARN("Robot stopped: Y limit.");
		}
		break;

	}
	
	ros::spinOnce();

	loop_rate.sleep();

  }

  return 0;
}
