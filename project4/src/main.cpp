//state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//parameters we should adjust : K, margin, MaxStep
// int margin = 5;
// int K = 500;
// double MaxStep = 2;
// int waypoint_margin = 24;
int margin = 7;
int K = 1000;
double MaxStep = 1.5;
int waypoint_margin = 24;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;

//FSM state
int state;

//function definition
void setcmdvel(double v, double w);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();

int main(int argc, char** argv){
    ros::init(argc, argv, "slam_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);
    ros::Subscriber gazebo_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);

    printf("Initialize topics\n");
    
    // FSM
    state = INIT;
    bool running = true;
    ros::Rate control_rate(60);
    int look_ahead_idx;

    double speed = 1.0;
    point current_goal;
    PID pid_ctrl;

    while(running && ros::ok()){
        printf("%d\n", state);
        switch (state) {
        case INIT: {
            
            // Load Map
            char* user = getpwuid(getuid())->pw_name;
            cv::Mat map_org = cv::imread((std::string("/home/") + std::string(user) +
                              std::string("/catkin_ws/src/project4/src/slam_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.5;
            world_x_max = 4.5;
            world_y_min = -13.5;
            world_y_max = 13.5;
            res = 0.05;
            printf("Load map\n");

             if(! map.data )                              // Check for invalid input
            {
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;
        } break;

        case PATH_PLANNING: {
            
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");
            
            // double RES = 2;
            // cv::Mat map_margin = map.clone();
            // int xSize = map_margin.cols;
            // int ySize = map_margin.rows;
            // for (int i = 0; i < ySize; i++) {
            //     for (int j = 0; j < xSize; j++) {
            //         if (map.at<uchar>(i, j) < 125) {
            //             for (int k = i - margin; k <= i + margin; k++) {
            //                 for (int l = j - margin; l <= j + margin; l++) {
            //                     if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
            //                         map_margin.at<uchar>(k, l) = 0;
            //                     }
            //                 }
            //             }
            //         }
            //     }
            // }
            // cv::Mat map_copy = map_margin.clone();
            // cv::cvtColor(map_copy, map_copy, CV_GRAY2BGR);
            // cv::resize(map_copy, map_copy, cv::Size(), 2, 2);
            // for(int i = 0; i < waypoints.size(); i++){
            //     std::vector<cv::Scalar> colors;
            //     colors.push_back(cv::Scalar(0, 0, 255));
            //     colors.push_back(cv::Scalar(0, 128, 255));
            //     colors.push_back(cv::Scalar(0, 255, 255));
            //     colors.push_back(cv::Scalar(0, 255, 0));
            //     colors.push_back(cv::Scalar(255, 0, 0));
            //     colors.push_back(cv::Scalar(255, 0, 128));
            //     printf("waypoints[%d]: (%f, %f)\n", i, waypoints[i].x, waypoints[i].y);
            //     int y = RES*(waypoints[i].x/res + map_origin_x);
            //     int x = RES*(waypoints[i].y/res + map_origin_y);
            //     cv::circle(map_copy, cv::Point(x, y), 5, colors[i % colors.size()], cv::FILLED, 8);
            // }
            // cv::namedWindow("waypoints");
            // cv::imshow("waypoints", map_copy);
            // cv::waitKey(0);
            // printf("map size: %d * %d\n", map_copy.rows, map_copy.cols);
            
            // RRT
            generate_path_RRT();
            printf("Generate RRT\n");

            int MAXIT = path_RRT.size();
            for (int i=0; i<MAXIT-1; i++){
                if(path_RRT[i].x == path_RRT[i+1].x && path_RRT[i].y == path_RRT[i+1].y){
                    path_RRT.erase(path_RRT.begin()+i);
                    MAXIT--;
                }
            }

            // double RES = 2;
            // cv::Mat map_margin = map.clone();
            // int xSize = map_margin.cols;
            // int ySize = map_margin.rows;
            // for (int i = 0; i < ySize; i++) {
            //     for (int j = 0; j < xSize; j++) {
            //         if (map.at<uchar>(i, j) < 125) {
            //             for (int k = i - margin; k <= i + margin; k++) {
            //                 for (int l = j - margin; l <= j + margin; l++) {
            //                     if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
            //                         map_margin.at<uchar>(k, l) = 0;
            //                     }
            //                 }
            //             }
            //         }
            //     }
            // }
            // cv::Mat map_copy = map_margin.clone();
            // cv::cvtColor(map_copy, map_copy, CV_GRAY2BGR);
            // cv::resize(map_copy, map_copy, cv::Size(), 2, 2);
            // std::vector<cv::Scalar> colors;
            // colors.push_back(cv::Scalar(0, 0, 255));
            // colors.push_back(cv::Scalar(0, 128, 255));
            // colors.push_back(cv::Scalar(0, 255, 255));
            // colors.push_back(cv::Scalar(0, 255, 0));
            // colors.push_back(cv::Scalar(255, 0, 0));
            // colors.push_back(cv::Scalar(255, 0, 128));

            // for(int i = 0; i < path_RRT.size(); i++){
            //     // printf("waypoints[%d]: (%f, %f)\n", i, waypoints[i].x, waypoints[i].y);
            //     int y = RES*(path_RRT[i].x/res + map_origin_x);
            //     int x = RES*(path_RRT[i].y/res + map_origin_y);
            //     cv::circle(map_copy, cv::Point(x, y), 5, colors[i % colors.size()], cv::FILLED, 8);
            // }
            // cv::namedWindow("path_RRT");
            // cv::imshow("path_RRT", map_copy);
            // cv::waitKey(0);
            
            look_ahead_idx = 0;
            current_goal.x = path_RRT[look_ahead_idx].x;
            current_goal.y = path_RRT[look_ahead_idx].y;
            current_goal.th = path_RRT[look_ahead_idx].th;

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            state = RUNNING;

        }

        case RUNNING: {
            //TODO
            printf("current goal: %d\n", look_ahead_idx);

            // generate ctrl by PID
            double ctrl = pid_ctrl.get_control(robot_pose, current_goal);

            // limit ctrl range
            if(fabs(ctrl) > 60.0 * M_PI / 180.0){ 
                printf("CTRL over range\n");
                if(ctrl > 0)
                    ctrl = 60.0 * M_PI / 180.0;
                else
                    ctrl = - 60.0 * M_PI / 180.0;
            }

            // calculate speed according to ctrl
                    
            speed = (0.1 - 2) / (50.0 * M_PI / 180.0) * fabs(ctrl) + 2; // speed ranges from 2 to 0.1, according to ctrl
            if(speed < 0.1)
                speed = 0.1;
                    
            setcmdvel(speed, ctrl);

            // step 2
            cmd_vel_pub.publish(cmd);

            // step 3
            if(distance(robot_pose, current_goal) < 0.2){
                    
                look_ahead_idx++;
                current_goal.x = path_RRT[look_ahead_idx].x;
                current_goal.y = path_RRT[look_ahead_idx].y;
                current_goal.th = path_RRT[look_ahead_idx].th;
                pid_ctrl.reset();

            }

            if (look_ahead_idx >= path_RRT.size())
                state = FINISH;

			ros::spinOnce();
			control_rate.sleep();

        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        } break;

        default: {
        } break;
        }
    }
    return 0;
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
    //printf("x,y : %f,%f \n",robot_pose.x,robot_pose.y);
}

void set_waypoints()
{
    std::srand(std::time(NULL));
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;

    cv::Mat map_margin = map.clone();
    int jSize = map.cols; // the number of columns
    int iSize = map.rows; // the number of rows

    for (int i = 0; i < iSize; i++) {
        for (int j = 0; j < jSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - waypoint_margin; k <= i + waypoint_margin; k++) {
                    for (int l = j - waypoint_margin; l <= j + waypoint_margin; l++) {
                        if (k >= 0 && l >= 0 && k < iSize && l < jSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    //TODO 2
    for (int i = 1; i < 4; i++){
        int random_x, random_y;
        while(1) {
            int min_x = (int(map_margin.cols - 1) / 2) * (i == 1);
            int max_x = min_x + int(map_margin.cols - 1) / 2;
            int min_y = (int(map_margin.rows - 1) / 2) * (i < 3);
            int max_y = min_y + int(map_margin.rows - 1) / 2;
            random_y = random_gen(min_x, max_x);
            random_x = random_gen(min_y, max_y);
            // printf("x, y: %f, %f\n", random_x, random_y);
            bool near = true;
            for (int j = -5; j < 3; j++){
                for (int k = -3; k < 3; k++){
                    if (map_margin.at<uchar>(random_x+j, random_y+k) < 50){
                        near = false;
                        break;
                    }
                    // if (map_margin.at<uchar>(random_x, random_y+k) < 50){
                    //     near = false;
                    //     break;
                    // }
                    // if (map_margin.at<uchar>(random_x+j, random_y+k) < 50){
                    //     near = false;
                    //     break;
                    // }
                    // if (map_margin.at<uchar>(random_x-j, random_y+k) < 50){
                    //     near = false;
                    //     break;
                    // }
                }
            }
            //if (map_margin.at<uchar>(random_x, random_y) > 200){
            if (near){
                printf("%d, %d\n", random_x, random_y);
                break;
            }
        }
        waypoint_candid[i].x = res * (random_x - map_origin_x);
        waypoint_candid[i].y = res * (random_y - map_origin_y);
    }

    // waypoint_candid[1].x = 2.0;
    // waypoint_candid[1].y = 12.0;
    // waypoint_candid[2].x = 3.5;
    // waypoint_candid[2].y = -10.5;
    // waypoint_candid[3].x = -2.0;
    // waypoint_candid[3].y = -12.0;

    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 12.0;

    int order[] = {0,1,2,3,4};
    int order_size = 5;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void generate_path_RRT()
{
    /*
     * 1. for loop
     * 2.  call RRT generate function in order to make a path which connects i way point to i+1 way point.
     * 3.  store path to variable "path_RRT"
     * 4.  when you store path, you have to reverse the order of points in the generated path since BACKTRACKING makes a path in a reverse order (goal -> start).
     * 5. end
     */


    point pre_leaf;
    double prev_th;
    double prev_th2;
    int count = 0;
    for(int i = 0; i < (waypoints.size() - 1); i++){ // initial point is waypoints[0]
        printf("to waypoint: %d/%d\n", i+1, waypoints.size());
        point x_init = waypoints[i];
        point x_goal = waypoints[i + 1];
        if (i > 0){
            //printf("positive i\n");
            x_init = pre_leaf;
            prev_th = atan2(x_goal.y - x_init.y, x_goal.x - x_init.x);
            double new_th = random_gen(prev_th - 0.3, prev_th + 0.3);
            x_init.th = new_th;
            

        }
        rrtTree *tree;
        tree = new rrtTree(x_init, x_goal, map, map_origin_x, map_origin_y, res, margin);
        printf("%d th tree\n", i);
        //printf("x_init: (%f, %f)\n", x_init.x, x_init.y);
        int valid = tree->generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
        while (valid == 0){
            count++;
            double max_alpha = 0.4;
            delete tree;
            if (i > 0) {
                if (count <= 10){
                    x_init.th = random_gen(prev_th2 - max_alpha, prev_th2 + max_alpha);
                }
                else if (count > 10){
                    x_init.th = random_gen(prev_th - max_alpha, prev_th + max_alpha);
                }
                if (count == 20){
                    count = 0;
                }
            }
            tree = new rrtTree(x_init, x_goal, map, map_origin_x, map_origin_y, res, margin);
            valid = tree->generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
        }
        tree->visualizeTree();
        printf("%d th generate\n", i);

        // Check plz
        std::vector<traj> temp_path = tree->backtracking_traj();
        printf("%d th backtrack\n", i);
        
        // tree->visualizeTree(temp_path);
        // tree->visualizeTree();
        // prev_th = atan2(x_goal.y - x_init.y, x_goal.x - x_init.x);
        
        prev_th2 = temp_path[0].th;

        pre_leaf = { temp_path[0].x, temp_path[0].y, 0 };
        traj tmp1 = temp_path[0];
        traj tmp2 = temp_path[temp_path.size()-1];
        // printf("path begin: (%f, %f), path end: (%f, %f)\n", tmp2.x, tmp2.y, tmp1.x, tmp1.y);
        // printf("size: %d\n", temp_path.size());
        
        // int iter_max = temp_path.size();
        // temp_path.pop_back(); // skip first point
        // for(int j = 0; j < iter_max; j++){
        //     path_RRT.push_back(temp_path.back());
        //     temp_path.pop_back();
        // }

        for(int j = temp_path.size() - 1; j >= 0; j--){
            path_RRT.push_back(temp_path[j]);
        }
        


        // printf("waypoint %d: %d\n", i+1, path_RRT.size());
        printf("%d th iteration\n", i);

        delete tree;
    }

}
