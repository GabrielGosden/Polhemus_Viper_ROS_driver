
#include "path_generation_functions.h"
#include "ros/ros.h"
#include <math.h>
#include <termios.h>
double d = 0, x_1 = 0, x_2 = 0, y_1 = 0,y_2 = 0, z_1 = 0, z_2 = 0, sample_number_1 = 0, sample_number_2 = 0, vel = 0 /*,time = 0*/;



double calc_velocity(double x, double y, double z,int sample_number){

/*
// Save the current pose and sample number
x_1 = x;
y_1 = y;
z_1 = z;
sample_number_1=sample_number;


// calculate the distance (Delta_d)
d=pow(pow(x_2-x_1,2)+pow(y_2-y_1,2)+pow(z_2-z_1,2),0.5);

// Calculate the time (Delta_t)
time=(sample_number_1-sample_number_2)*0.0042;

// Calculate the velocity (Vel=Delta_d/Delta_t)
vel=d/time;


// Save the current pose and sample number as the old one.
x_2 = x_1;
y_2 = y_1;
z_2 = z_1;
sample_number_2=sample_number_1;


return vel;
*/
}


char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv == 0)
        d=0;
        //ROS_INFO("no_key_pressed");

    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}

