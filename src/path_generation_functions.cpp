#include "path_generation_functions.h"
#include <math.h>
double d = 0, x_1 = 0, x_2 = 0, y_1 = 0,y_2 = 0, z_1 = 0, z_2 = 0, sample_number_1 = 0, sample_number_2 = 0, vel = 0, time = 0;



double calc_velocity(double x, double y, double z,int sample_number){


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

}





