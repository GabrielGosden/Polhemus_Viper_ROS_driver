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
#include "Polhemus_Viper_ROS_Driver/viper_msg_pose_ori.h"
#include "kalman_functions.h"


#include <sstream>


double x,y,z,az,el,ro;
int kalman_sample_time;
void outlier_detection_pose_ori_callback(const Polhemus_Viper_ROS_Driver::viper_msg_pose_ori  &msg)
{
    x = msg.x;
    y = msg.y;
    z = msg.z;
    az = msg.az;
    el = msg.el;
    ro = msg.ro;
   // ROS_INFO("x=%f, y=%f, z=%f, az=%f, el=%f, ro=%f",x,y,z,az,el,ro);
}

/*
double kalman_pose_x(double U){
    static double R = 1;                                   // Noise covariance
    static double H = 1;                                   // Measurement map scalar
    static double Q = 0.1;                                 // Initial estimated covariance
    static double P = 0.1;                                 // Initial error covariance
    static double U_HAT = 0;                               // Inital estimated state
    static double K = 0;                                   // Initial kalman gain

    K=P*H/(H*P*H+R);
    U_HAT=U_HAT+K*(U-H*U_HAT);
    P=(1-K*H)*P+Q;
    
    return U_HAT;
}
*/
int main(int argc, char **argv)
{

ros::init(argc, argv, "kalman_filter");

ros::NodeHandle n;

ros::Subscriber sub = n.subscribe("outlier_detection_pose_ori",10,outlier_detection_pose_ori_callback);

ros::Publisher kalman_filter_pose_ori= n.advertise<Polhemus_Viper_ROS_Driver::viper_msg_pose_ori>("kalman_filter_pose_ori", 1000);


n.getParam("/path_sample_time", kalman_sample_time);
ros::Rate loop_rate(kalman_sample_time);


Polhemus_Viper_ROS_Driver :: viper_msg_pose_ori msg_pose_ori;



double x_filtered, y_filtered, z_filtered, az_filtered,el_filtered,ro_filtered, init = 0;  

ros::Duration(2.0).sleep();

//ROS_INFO("Kalman filtering started!");

while (ros::ok())
  {
  msg_pose_ori.x = x;
	msg_pose_ori.y = y;
	msg_pose_ori.z = z;
	msg_pose_ori.az = az;
	msg_pose_ori.el = el;
	msg_pose_ori.ro = ro;
  /*
  msg_pose_ori.x = kalman_pose_x(x);
	msg_pose_ori.y = kalman_pose_y(y);
	msg_pose_ori.z = kalman_pose_z(z);
	msg_pose_ori.az = kalman_ori_az(az);
	msg_pose_ori.el = kalman_ori_el(el);
	msg_pose_ori.ro = kalman_ori_ro(ro);*/
  //ROS_INFO("x=%f, y=%f, z=%f, az=%f, el=%f, ro=%f",x,y,z,az,el,ro);
  //ROS_INFO("message = x=%f, y=%f, z=%f, az=%f, el=%f, ro=%f",msg_pose_ori.x,msg_pose_ori.y,msg_pose_ori.z,msg_pose_ori.az,msg_pose_ori.el,msg_pose_ori.ro);
	kalman_filter_pose_ori.publish(msg_pose_ori);

    if(init == 0 && msg_pose_ori.x != 0){
        ROS_INFO("Kalman filtering started!");
        init = 1;
    }
    
    ros::spinOnce();
   
    loop_rate.sleep();

  }


  return 0;
}