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
    for(i = 0; i < data_size; ++i)
    {
        sum += data[i];
    }
    mean = sum/data_size;

    for(i = 0; i < data_size; ++i)
        standardDeviation += pow(data[i] - mean, 2);

    return sqrt(standardDeviation / data_size);
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
    
    float data_x[data_size], data_y[data_size], data_z[data_size], data_az[data_size], data_el[data_size], data_ro[data_size];
    double std_x, std_y, std_z, std_az, std_el, std_ro;
    double x_min, x_max, x_avg, x_sum;
    double y_min, y_max, y_avg, y_sum;
    double z_min, z_max, z_avg, z_sum;
    double az_min, az_max, az_avg, az_sum;
    double el_min, el_max, el_avg, el_sum;
    double ro_min, ro_max, ro_avg, ro_sum;
    int frame_good = 0, frame_bad = 0;
    double drop_rate = 0;
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
            for(int i = 0;i < data_size; i++){
                if( i== 0){
                    x_sum = 0;
                    y_sum = 0;
                    z_sum = 0;
                    az_sum = 0;
                    el_sum = 0;
                    ro_sum = 0;
                }
                x_sum = x_sum+data_x[i];
                y_sum = y_sum+data_y[i];
                z_sum = z_sum+data_z[i];
                az_sum = az_sum+data_az[i];
                el_sum = el_sum+data_el[i];
                ro_sum = ro_sum+data_ro[i];
            }
            
            x_avg = x_sum/data_size;
            y_avg = y_sum/data_size;
            z_avg = z_sum/data_size;

            az_avg = az_sum/data_size;
            el_avg = el_sum/data_size;
            ro_avg = ro_sum/data_size;

            if(x_avg > x_min && x_avg < x_max){
                if(y_avg > y_min && y_avg < y_max){
                    if(z_avg > z_min && z_avg < z_max){
                        if(az_avg > az_min && az_avg < az_max){
                            if(el_avg > el_min && el_avg < el_max){
                                if(ro_avg > ro_min && ro_avg < ro_max){
                                    frame_good++;
                                    drop_rate = (float)frame_bad/((float)frame_good+(float)frame_bad);
                                    msg_pose_ori.x = x_avg;
	                                msg_pose_ori.y = y_avg;
	                                msg_pose_ori.z = z_avg;
	                                msg_pose_ori.az = az_avg;
	                                msg_pose_ori.el = el_avg;
	                                msg_pose_ori.ro = ro_avg;
                                    outlier_detection_pose_ori.publish(msg_pose_ori);
                                    //ROS_INFO("Sample okay! Drop rate is :%f percent",drop_rate*100); 
                                }else{
                                    frame_bad++;
                                    //ROS_INFO("Sample ro NOT okay! %d",frame_bad);
                                }
                            }else{
                                frame_bad++;
                                //ROS_INFO("Sample el NOT okay! %d",frame_bad);
                            }
                        }else{
                            frame_bad++;
                            //ROS_INFO("Sample az NOT okay! %d",frame_bad);
                        }

                    }else{
                        frame_bad++;
                        //ROS_INFO("Sample z NOT okay! %d",frame_bad);
                    }
                }else{
                    frame_bad++;
                    //ROS_INFO("Sample y NOT okay! %d",frame_bad);
                }
            }else{
                frame_bad++;
                //ROS_INFO("Sample x NOT okay! %d",frame_bad);
            }

        //ROS_INFO("%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f ",x_min, x_max, x_avg, y_min, y_max, y_avg, z_min, z_max, z_avg,el_min, el_max, el_avg, az_min, az_max, az_avg, ro_min, ro_max, ro_avg);
        
        // Calculate standard deviation    
        std_x = calculateSD(data_x,data_size);
        std_y = calculateSD(data_y,data_size);
        std_z = calculateSD(data_z,data_size);
        std_az = calculateSD(data_az,data_size);
        std_el = calculateSD(data_el,data_size);
        std_ro = calculateSD(data_ro,data_size);
        x_min = x - 3*std_x;
        x_max = x + 3*std_x;
        y_min = y - 3*std_y;
        y_max = y + 3*std_y;
        z_min = z - 3*std_z;
        z_max = z + 3*std_z;
        az_min = az - 3*std_az;
        az_max = az + 3*std_az;
        el_min = el - 3*std_el;
        el_max = el + 3*std_el;
        ro_min = ro - 3*std_ro;
        ro_max = ro + 3*std_ro;
         
        // After calculation update sample number
        sample_number_old= sample_number;


  

        }
    
        ros::spinOnce();

        loop_rate.sleep();

  }


  return 0;
}