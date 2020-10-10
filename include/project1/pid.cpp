#include <project1/pid.h>
#include <stdio.h>

PID::PID(){

    /* TO DO
     *
     * find appropriate value of the coefficients for PID control, and initialize them.
     *
    */

    /////////////////// MY CODE BEGIN ///////////////////

    // init errors
    error = 0;
    error_sum = 0;
    error_diff = 0;

    // init gains 
    // THINK: You should adjust these values for precise control
    Kp = 1;
    Ki = 0.5;
    Kd = 0;

    /////////////////// MY CODE END ///////////////////

}

float PID::get_control(point car_pose, point goal_pose){

    float ctrl;

    /* TO DO
     *
     * implement pid algorithm
     *
    */

    /////////////////// MY CODE BEGIN ///////////////////

    // copy member variables from the struct arguments
    // THINK: Only th value is used, is it ok?
    double car_x, car_y, car_th;
    double goal_x, goal_y, goal_th;

    car_x = car_pose.x;
    car_y = car_pose.y;
    car_th = car_pose.th;

    goal_x = goal_pose.x;
    goal_y = goal_pose.y;
    //goal_th = goal_pose.th;

    goal_th = atan2((goal_y - car_y) , (goal_x - car_x));
    printf("car_th: %.2f, goal_th: %.2f\n", car_th, goal_th);
    

    // control rate: 10Hz (refer to pidmain.cpp)
    // THINK: Is it okay to regard control rate as a constant value? 
    double rate = 10;

    // error computation and update
    float e_t; // temp error variable, for error difference computation

    e_t = (float)(goal_th - car_th);
    error_diff = e_t - error;
    //error_sum += e_t; // gyuri
    error = e_t;

    // final ctrl computation
    ctrl = Kp * error + Ki / rate * error_sum + Kd * rate * error_diff;
    error_sum += e_t;  // gyuri

    /////////////////// MY CODE END ///////////////////


    return ctrl;
}
