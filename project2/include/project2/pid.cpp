#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

PID::PID(){

    /* TO DO
     *
     * find appropriate values for PID contorl, and initialize them.
     * 
     * Currently, the code is copied from Project1
     *
    */

    // init errors
    error = 0;
    error_sum = 0;
    error_diff = 0;

    // init gains 
    Kp = 1.0;
    Ki = 0.1;
    Kd = 0.1;
}

void PID::reset() {
    error = 0;
    error_sum = 0;
    error_diff = 0;
}

float PID::get_control(point car_pose, point goal_pose){

    float ctrl;

    /* TO DO
     *
     * implement pid algorithm
     * 
     * Currently, the code is copied from Project1
     *
    */

    // copy member variables

    double car_x, car_y, car_th;
    double goal_x, goal_y, goal_th;

    car_x = car_pose.x;
    car_y = car_pose.y;
    car_th = car_pose.th;

    goal_x = goal_pose.x;
    goal_y = goal_pose.y;

    goal_th = atan2((goal_y - car_y) , (goal_x - car_x));
    

    // control rate: 10Hz (refer to pidmain.cpp)
    double rate = 60;

    // error computation and update
    float e_t; // temp error variable, for error difference computation

    e_t = (float)(goal_th - car_th);
    if (e_t > M_PI){
        e_t = -(2 * M_PI - e_t);
    }
    else if(e_t < - M_PI){
        e_t = 2 * M_PI + e_t;
    }

    error_diff = e_t - error;
    error = e_t;

    // final ctrl computation
    ctrl = Kp * error + Ki / rate * error_sum + Kd * rate * error_diff;

    error_sum += e_t;

    return ctrl;
}

