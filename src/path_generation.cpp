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
#include <Polhemus_Viper_ROS_Driver/viper_msg_pose_ori.h>
#include <Polhemus_Viper_ROS_Driver/viper_msg_n.h>
#include <Polhemus_Viper_ROS_Driver/viper_msg_dist.h>



//Global variables
double x,y,z,az,el,ro;
int dist,state = 1, init_run = 0;
char input_string[20];

// Variables for .launch files
int sample_time, dist_limit;
double sensor_x_max,sensor_x_min,sensor_y_max,sensor_y_min,sensor_z_max,sensor_z_min,sensor_az_max,sensor_az_min,sensor_el_max,sensor_el_min,sensor_ro_max,sensor_ro_min;
double robot_x_max,robot_x_min,robot_y_max,robot_y_min,robot_z_max,robot_z_min,robot_az_max,robot_az_min,robot_el_max,robot_el_min,robot_ro_max,robot_ro_min;
double ur_gain, ur_time, ur_lookahead_time;


// Test variables. Delete later
double temp;

// Define ROS publisher globally to avoid scope issues
ros::Publisher ur_script_pub;


// Callback function for kalman_filter_pose_ori_sub
void kalman_filter_pose_ori_callback(const Polhemus_Viper_ROS_Driver::viper_msg_pose_ori  &msg)
{	
	//Apply Rotation matrix directly (Rot(x,pi)*Rot(z,pi/2))
    x = msg.y/100;
    y = msg.x/100;
    z = (msg.z/100)*(-1);
    az = msg.az*(-1);
    el = msg.el;
    ro = msg.ro;
}


// Callback function for viper_broadcaster_dist_sub
void viper_broadcaster_dist_callback(const Polhemus_Viper_ROS_Driver::viper_msg_dist  &msg)
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
	// Get distortion level from the viper
	ros::Subscriber viper_broadcaster_dist_sub = n.subscribe("viper_broadcaster_dist",10,viper_broadcaster_dist_callback);
	// Publisher for movel to UR
	//ros::Publisher ur_script_pub = n.advertise<std_msgs::String>("ur_hardware_interface/script_command",10);
	
	//Get params from .yaml files.
	n.getParam("/path_sample_time", sample_time);
	n.getParam("/sensor_x_max", sensor_x_max);
	n.getParam("/sensor_x_min", sensor_x_min);
	n.getParam("/sensor_y_max", sensor_y_max);
	n.getParam("/sensor_y_min", sensor_y_min);
	n.getParam("/sensor_z_max", sensor_z_max);
	n.getParam("/sensor_z_min", sensor_z_min);
	n.getParam("/sensor_az_max", sensor_az_max);
	n.getParam("/sensor_az_min", sensor_az_min);
	n.getParam("/sensor_el_max", sensor_el_max);
	n.getParam("/sensor_el_min", sensor_el_min);
	n.getParam("/sensor_ro_max", sensor_ro_max);
	n.getParam("/sensor_ro_min", sensor_ro_min);
	n.getParam("/robot_x_max", robot_x_max);
	n.getParam("/robot_x_min", robot_x_min);
	n.getParam("/robot_y_max", robot_y_max);
	n.getParam("/robot_y_min", robot_y_min);
	n.getParam("/robot_z_max", robot_z_max);
	n.getParam("/robot_z_min", robot_z_min);
	n.getParam("/robot_az_max", robot_az_max);
	n.getParam("/robot_az_min", robot_az_min);
	n.getParam("/robot_el_max", robot_el_max);
	n.getParam("/robot_el_min", robot_el_min);
	n.getParam("/robot_ro_max", robot_ro_max);
	n.getParam("/robot_ro_min", robot_ro_min);
	n.getParam("/dist_limit", dist_limit);
	n.getParam("/ur_time", ur_time);
	n.getParam("/ur_lookahead_time", ur_lookahead_time);
	n.getParam("/ur_gain", ur_gain);
	

	// Set ROS loop_rate
	
	ros::Rate loop_rate(sample_time);
	//ros::Rate loop_rate(80);

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
			//ros::Duration(2.5).sleep();
			ROS_INFO("Sending robot to starting position, please wait.");
			state=2;

		// Confirm by user that robot is at safe starting position.
		case 2:
			ROS_INFO("Confirm robot is at starting position by pressing 'Enter'.");
			fgets(input_string,100,stdin);
			state=3;
			//state=4;
		break;
		
		// Move sensor to safe starting position.
		case 3:
			ROS_INFO("x = %f, y = %f, z = %f, az = %f, el = %f, ro = %f",x,y,z,az,el,ro);
			 
			ROS_INFO("Please move Viper sensor to the safe starting position and orientation");
			if(x > sensor_x_min && x < sensor_x_max){
				if(y > sensor_y_min && y < sensor_y_max){
					if(z > sensor_y_min && z < sensor_y_max){
						if(az > sensor_az_min && az < sensor_az_max){
							if(el > sensor_el_min && el < sensor_el_max){
								if(ro > sensor_ro_min && ro < sensor_ro_max){
									ROS_INFO("Sensor is in safe starting position.");
									state = 4;
								}else{
									ROS_INFO("Sensor Ro limit.");
								}
							}else{
								ROS_INFO("Sensor El limit.");
							}
						}else{
							ROS_INFO("Sensor Az limit.");
						}
					}else{
						ROS_INFO("Sensor Z limit.");
					}
				}else{
					ROS_INFO("Sensor Y limit.");
				}
			}else{
				ROS_INFO("Sensor X limit.");
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

		// Create UR script command publisher.
		if (init_run = 0){
			ur_script_pub = n.advertise<std_msgs::String>("ur_hardware_interface/script_command",10);
			ros::Duration(0.5).sleep();
			init_run = 1;
		}
		
		//ROS_INFO("x = %f, y = %f, z = %f, az = %f, el = %f, ro = %f",x,y,z,az,el,ro);
		//ROS_INFO("robot_x_min = %f, robot_x_max = %f",robot_x_min,robot_x_max);
		if(x > robot_x_min && x < robot_x_max){
			if(y > robot_y_min && y < robot_y_max){
				if(z > robot_z_min && z < robot_z_max){
					if(az > robot_az_min && az < robot_az_max){
						if(el > robot_el_min && el < robot_el_max){
							if(ro > robot_ro_min && ro < robot_ro_max){
								if(dist < dist_limit){
									//ss << "servoj(get_inverse_kin(p[-0.217,0.553,0.496,3.72,-0.3,-0.12]), t=" << ur_time << ", lookahead_time=" << ur_lookahead_time << ", gain=" << ur_gain << ")";
									ss << "servoj(get_inverse_kin(p[" << x << "," << y << "," << z << "," << el << "," << ro << "," << az << "]), t=" << ur_time << ", lookahead_time=" << ur_lookahead_time << ", gain=" << ur_gain << ")";
									msg.data = ss.str();
									ur_script_pub.publish(msg);
									ROS_INFO("Within limits, sending command");
								}else{
									ROS_WARN("Robot stopped: Distorition limit");
								}
							}else{
								ROS_WARN("Robot stopped: Ro limit.");
							}
						}else{
							ROS_WARN("Robot stopped: El limit.");
						}
					}else{
						ROS_WARN("Robot stopped: Az limit.");
					}
				}else{
					ROS_WARN("Robot stopped: Z limit.");
				}
			}else{
				ROS_WARN("Robot stopped: Y limit.");
			}
		}else{
			ROS_WARN("Robot stopped: X limit.");
		}
		break;

	}
	
	ros::spinOnce();

	loop_rate.sleep();

  }

  return 0;
}
