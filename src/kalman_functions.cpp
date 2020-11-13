
#include "kalman_functions.h"

static const double R = 1;                                   // Noise covariance
static const double H = 1;                                   // Measurement map scalar


double kalman_pose_x(double U){

    static double Q = 0.1;                                 // Initial estimated covariance
    static double P = 0.1;                                 // Initial error covariance
    static double U_HAT = 0;                               // Inital estimated state
    static double K = 0;                                   // Initial kalman gain

    K=P*H/(H*P*H+R);
    U_HAT=U_HAT+K*(U-H*U_HAT);
    P=(1-K*H)*P+Q;
    
    return U_HAT;
}
double kalman_pose_y(double U){

    static double Q = 0.1;                                 // Initial estimated covariance
    static double P = 0.1;                                 // Initial error covariance
    static double U_HAT = 0;                               // Inital estimated state
    static double K = 0;                                   // Initial kalman gain

    K=P*H/(H*P*H+R);
    U_HAT=U_HAT+K*(U-H*U_HAT);
    P=(1-K*H)*P+Q;
    
    return U_HAT;
}
double kalman_pose_z(double U){

    static double Q = 0.1;                                 // Initial estimated covariance
    static double P = 0.1;                                 // Initial error covariance
    static double U_HAT = 0;                               // Inital estimated state
    static double K = 0;                                   // Initial kalman gain

    K=P*H/(H*P*H+R);
    U_HAT=U_HAT+K*(U-H*U_HAT);
    P=(1-K*H)*P+Q;
    
    return U_HAT;
}
double kalman_ori_az(double U){
    static double Q = 0.1;                                 // Initial estimated covariance
    static double P = 0.1;                                 // Initial error covariance
    static double U_HAT = 0;                               // Inital estimated state
    static double K = 0;                                   // Initial kalman gain

    K=P*H/(H*P*H+R);
    U_HAT=U_HAT+K*(U-H*U_HAT);
    P=(1-K*H)*P+Q;
    
    return U_HAT;
}

double kalman_ori_el(double U){

    static double Q = 0.1;                                 // Initial estimated covariance
    static double P = 0.1;                                 // Initial error covariance
    static double U_HAT = 0;                               // Inital estimated state
    static double K = 0;                                   // Initial kalman gain

    K=P*H/(H*P*H+R);
    U_HAT=U_HAT+K*(U-H*U_HAT);
    P=(1-K*H)*P+Q;
    
    return U_HAT;
}
double kalman_ori_ro(double U){

    static double Q = 0.1;                                 // Initial estimated covariance
    static double P = 0.1;                                 // Initial error covariance
    static double U_HAT = 0;                               // Inital estimated state
    static double K = 0;                                   // Initial kalman gain

    K=P*H/(H*P*H+R);
    U_HAT=U_HAT+K*(U-H*U_HAT);
    P=(1-K*H)*P+Q;
    
    return U_HAT;
}


