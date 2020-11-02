#include "ros/ros.h"
#include "std_msgs/String.h"
// Include the custom messages.
#include "viper_tf2_broadcaster/viper_msg_pose_ori.h"
#include "viper_tf2_broadcaster/viper_msg_n.h"
//Include the path generation functions.
#include "path_generation_functions.h"


/* Global variables*/
double x,y,z,az,el,ro;
int sample_number, sample_time;

// Callback function for kalman_filter_pose_ori_sub
void kalman_filter_pose_ori_callback(const viper_tf2_broadcaster::viper_msg_pose_ori  &msg)
{
    x = msg.x;
    y = msg.y;
    z = msg.z;
    az = msg.az;
    el = msg.el;
    ro = msg.ro;
}
// Callback function for viper_broadcaster_n_sub
void viper_broadcaster_n_callback(const viper_tf2_broadcaster::viper_msg_n  &msg)
{
   sample_number = msg.n;
   
}


double vel;

int main(int argc, char **argv)
{
  
	ros::init(argc, argv, "path_generation");

	ros::NodeHandle n;
	// Get filtered position and orientation from the kalman filter.
	ros::Subscriber kalman_filter_pose_ori_sub = n.subscribe("kalman_filter_pose_ori",10,kalman_filter_pose_ori_callback);
	// Get the sample number from the viper.
	ros::Subscriber viper_broadcaster_n_sub = n.subscribe("viper_broadcaster_n",10,viper_broadcaster_n_callback);

	
	n.getParam("/path_sample_time", sample_time);
	ros::Rate loop_rate(sample_time);

    while (ros::ok())
  {
	// Get velocity in cm/s 
	vel = calc_velocity(x, y, z, sample_number);

	//ROS_INFO("%f",vel);


	ros::spinOnce();

	loop_rate.sleep();

  }

  return 0;
}
