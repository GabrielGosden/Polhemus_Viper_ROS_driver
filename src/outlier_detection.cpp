#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Polhemus_Viper_ROS_Driver/viper_msg_pose_ori.h"
#include "Polhemus_Viper_ROS_Driver/viper_msg_n.h"
#include <sstream>



double x,y,z,az,el,ro;
int sample_number, init;

void viper_broadcaster_pose_ori_callback(const Polhemus_Viper_ROS_Driver::viper_msg_pose_ori  &msg)
{
    x = msg.x;
    y = msg.y;
    z = msg.z;
    az = msg.az;
    el = msg.el;
    ro = msg.ro;
}

// Callback function for viper_broadcaster_n_sub
void viper_broadcaster_n_callback(const Polhemus_Viper_ROS_Driver::viper_msg_n  &msg)
{
   sample_number = msg.n;
}


float calculateSD(float data[],int data_size)
{
    float sum = 0.0, mean, standardDeviation = 0.0;

    int i;
    //ROS_INFO("Start sum and size is %d", data_size);
    for(i = 0; i < data_size; ++i)
    {
        sum += data[i];
        //ROS_INFO("partial sum is %f", data[i]);
    }
    //ROS_INFO("End sum");
    mean = sum/data_size;

    for(i = 0; i < data_size; ++i)
        standardDeviation += pow(data[i] - mean, 2);

    return sqrt(standardDeviation / data_size);
    //return sum;
}



int main(int argc, char **argv)
{

    int outlier_detection_sample_time , path_generation_sample_time, data_size,sample_number_old,sample_number_current;

    ros::init(argc, argv, "outlier_detection");

    
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("viper_broadcaster_pose_ori",10,viper_broadcaster_pose_ori_callback);
    // Get the sample number from the viper.
	ros::Subscriber viper_broadcaster_n_sub = n.subscribe("viper_broadcaster_n",10,viper_broadcaster_n_callback);
    ros::Publisher outlier_detection_pose_ori = n.advertise<Polhemus_Viper_ROS_Driver::viper_msg_pose_ori>("outlier_detection_pose_ori", 10);
    

    n.getParam("/viper_sample_time", outlier_detection_sample_time);
    n.getParam("/path_sample_time", path_generation_sample_time);

    // Create arrays according to sample speed.
    data_size = outlier_detection_sample_time/path_generation_sample_time;
    //data_size = 10;
    
    float data_x[data_size], data_y[data_size], data_z[data_size], data_az[data_size], data_el[data_size], data_ro[data_size];

    double std_x, std_y, std_z, std_az, std_el, std_ro;
    

    ros::Rate loop_rate(outlier_detection_sample_time);


    Polhemus_Viper_ROS_Driver :: viper_msg_pose_ori msg_pose_ori;

    ros::Duration(1.0).sleep();
    while (ros::ok())
    {
        if (init == 0){
            sample_number_old=sample_number;
            //float test[6] = {1,2,1,2,1,3};
            //std_x = calculateSD(test,data_size);
            //ROS_INFO("test = %f",std_x);
            init = 1;
            //ROS_INFO("here1");
        }
        
        sample_number_current = sample_number;

        // Load data into array
        if(sample_number_current-sample_number_old <= data_size){
            //ROS_INFO("putting into place %d",sample_number_current-sample_number_old-1);
            data_x[sample_number_current-sample_number_old-1] = x;
            /*data_y[sample_number_current-sample_number_old] = y;
            data_z[sample_number_current-sample_number_old] = z;
            data_az[sample_number_current-sample_number_old] = az;
            data_el[sample_number_current-sample_number_old] = el;
            data_ro[sample_number_current-sample_number_old] = ro;*/
            //ROS_INFO("Loaded into arrays");
        }
       
        // Everytime array is full find standard deviation.
        if (sample_number_current-sample_number_old > data_size-1){
            std_x = calculateSD(data_x,data_size);
            /*std_y = calculateSD(data_y,data_size);
            std_z = calculateSD(data_z,data_size);
            std_az = calculateSD(data_az,data_size);
            std_el = calculateSD(data_el,data_size);
            std_ro = calculateSD(data_ro,data_size);*/
            //ROS_INFO("std_x = %f", std_x);
            //ROS_INFO("std_x = %f, std_y = %f, std_z = %f, std_az = %f, std_el = %f, std_ro = %f",std_x,std_y,std_z,std_az,std_el,std_ro);

            sample_number_old= sample_number;
  

        }

        /*

        msg_pose_ori.x = x;
        msg_pose_ori.y = y;
        msg_pose_ori.z = z;
        msg_pose_ori.az = az;
        msg_pose_ori.el = el;
        msg_pose_ori.ro = ro;
        outlier_detection_pose_ori.publish(msg_pose_ori);
*/
    
        ros::spinOnce();

        loop_rate.sleep();

  }


  return 0;
}