#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Polhemus_Viper_ROS_Driver/viper_msg_pose_ori.h"
#include "Polhemus_Viper_ROS_Driver/viper_msg_n.h"
#include <sstream>



double x,y,z,az,el,ro;
int sample_number, init;

// Callback function for viper_broadcaster_pose_ori
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

// Function to calculate Standard Deviation
float calculateSD(float data[],int data_size)
{
    float sum = 0.0, mean = 0.0, standardDeviation = 0.0;
    int i;
    for(i = 0; i < data_size; ++i)
    {
        sum += data[i];
    }
    mean = sum/data_size;

    for(i = 0; i < data_size; ++i)
        standardDeviation += pow(data[i] - mean, 2);

    return sqrt(standardDeviation / data_size);
}

// Function to calculate mean
float calculateMean(float data[],int data_size){
    float sum = 0.0, mean = 0.0;
    int i;
    for(i = 0; i < data_size; ++i)
    {
        sum += data[i];
    }
    mean = sum/data_size;

    return mean;
}


int main(int argc, char **argv)
{

    int outlier_detection_sample_time , path_generation_sample_time, data_size,sample_number_old,sample_number_current;

    ros::init(argc, argv, "outlier_detection");

    
    ros::NodeHandle n;

    // Get the frame from the viper.
    ros::Subscriber sub = n.subscribe("viper_broadcaster_pose_ori",10,viper_broadcaster_pose_ori_callback);

    // Get the sample number from the viper.
	ros::Subscriber viper_broadcaster_n_sub = n.subscribe("viper_broadcaster_n",10,viper_broadcaster_n_callback);

    //Publisher for outlier detection.
    ros::Publisher outlier_detection_pose_ori = n.advertise<Polhemus_Viper_ROS_Driver::viper_msg_pose_ori>("outlier_detection_pose_ori", 10);
    
    // Get the sample times
    n.getParam("/viper_sample_time", outlier_detection_sample_time);
    n.getParam("/path_sample_time", path_generation_sample_time);

    // Create arrays according to sample speed.
    data_size = outlier_detection_sample_time/path_generation_sample_time;

    // Array for input data
    float data_x[data_size], data_y[data_size], data_z[data_size], data_az[data_size], data_el[data_size], data_ro[data_size];
    // Arrays for calculation and output
    float std[6], mean[6], out_n[6], out[6];

    ros::Rate loop_rate(outlier_detection_sample_time);


    Polhemus_Viper_ROS_Driver :: viper_msg_pose_ori msg_pose_ori;

    ros::Duration(1.0).sleep();

    while (ros::ok())
    {
        // Init for correct array indexing
        if (init == 0){
            sample_number_old=sample_number;
            init = 1;
        }

        // If viper sample number overflows back to 0.
        if(sample_number == 0){
           sample_number_current = sample_number;
           sample_number_old = 0; 
        }else{
            sample_number_current = sample_number;
        }
 
        // Load data into array if array is not full.
        if(sample_number_current-sample_number_old <= data_size){
            data_x[sample_number_current-sample_number_old-1] = x;
            data_y[sample_number_current-sample_number_old-1] = y;
            data_z[sample_number_current-sample_number_old-1] = z;
            data_az[sample_number_current-sample_number_old-1] = az;
            data_el[sample_number_current-sample_number_old-1] = el;
            data_ro[sample_number_current-sample_number_old-1] = ro;
        }
       
        // Everytime array is full.
        if (sample_number_current-sample_number_old > data_size-1){

        // Reset output and output index
        for(int j = 0; j < data_size; j++){
        out[j] = 0;
        out_n[j] = 0;
        }

        // Calculate standard deviation for whole array 
        std[0] = calculateSD(data_x,data_size);
        std[1] = calculateSD(data_y,data_size);
        std[2] = calculateSD(data_z,data_size);
        std[3] = calculateSD(data_az,data_size);
        std[4] = calculateSD(data_el,data_size);
        std[5] = calculateSD(data_ro,data_size);

        // Calculate mean for whole array
        mean[0] = calculateMean(data_x, data_size);
        mean[1] = calculateMean(data_y, data_size);
        mean[2] = calculateMean(data_z, data_size);
        mean[3] = calculateMean(data_az, data_size);
        mean[4] = calculateMean(data_el, data_size);
        mean[5] = calculateMean(data_ro, data_size);   

        // Compare max and min to every sample in array
            for(int i = 0; i < data_size; i++){
                if(data_x[i] > mean[0] - 3*std[0] && data_x[i] < mean[0] + 3*std[0]){
                    out[0] += data_x[i];
                    out_n[0]++;
                }  
                if(data_y[i] > mean[1] - 3*std[1] && data_y[i] < mean[1] + 3*std[1]){
                    out[1] += data_y[i];
                    out_n[1]++;
                }
                if(data_z[i] > mean[2] - 3*std[2] && data_z[i] < mean[2] + 3*std[2]){
                    out[2] += data_z[i];
                    out_n[2]++;
                }
                if(data_az[i] > mean[3] - 3*std[3] && data_az[i] < mean[3] + 3*std[3]){
                    out[3] += data_az[i];
                    out_n[3]++;
                }
                if(data_el[i] > mean[4] - 3*std[4] && data_el[i] < mean[4] + 3*std[4]){
                    out[4] += data_el[i];
                    out_n[4]++;
                }
                if(data_ro[i] > mean[5] - 3*std[5] && data_ro[i] < mean[5] + 3*std[5]){
                    out[5] += data_ro[i];
                    out_n[5]++;
                }
                
            }
        // Incorporate skipped frames and create message    
        msg_pose_ori.x = out[0]/out_n[0];
	    msg_pose_ori.y = out[1]/out_n[1];
	    msg_pose_ori.z = out[2]/out_n[2];
	    msg_pose_ori.az = out[3]/out_n[3];
	    msg_pose_ori.el = out[4]/out_n[4];
	    msg_pose_ori.ro = out[5]/out_n[5];
        
        // Publish message
        outlier_detection_pose_ori.publish(msg_pose_ori);

        // Update sample number
        sample_number_old= sample_number;

        }
    
        ros::spinOnce();

        loop_rate.sleep();

  }


  return 0;
}