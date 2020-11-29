#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846

#define MAX_TABLE 20000

double max_alpha = 0.2; // maximum steering angle of front wheels --> alpha is in -max_alpha ~ max_alpha
//double max_alpha = 0.35;
double L = 0.325; // length of the RC car

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal) { 

    // This is already implemented, Just use in main.cpp

    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
}

rrtTree::~rrtTree(){
    printf("count: %d\n", count);
    for (int i = 0; i < count; i++) {
        delete ptrTable[i];
    }
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);
    
    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	    for(int j = 0; j < 10; j++) {
	        double alpha = this->ptrTable[i]->alpha;
	        double d = this->ptrTable[i]->d;
	        double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	        double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	        double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }
    // cv::namedWindow("Mapping");
    // cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    // cv::imshow("Mapping", imgResult(imgROI));
    // cv::waitKey(1);
    cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(0);
}

void rrtTree::visualizeTree(std::vector<traj> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	    for(int j = 0; j < 10; j++) {
	        double alpha = this->ptrTable[i]->alpha;
	        double d = this->ptrTable[i]->d;
	        double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	        double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	        double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
	    for(int j = 0; j < 10; j++) {
	        double alpha = path[i].alpha;
	        double d = path[i].d;
	        double p1_th = path[i-1].th + d*j/10*tan(alpha)/L; // R = L/tan(alpha)
            double p2_th = path[i-1].th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
	        double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
            double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
	        double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }
    // cv::namedWindow("Mapping");
    // cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    // cv::imshow("Mapping", imgResult(imgROI));
    // cv::waitKey(1);

    cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);

}
// Done
void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {

    // add new vertex to tree
    // tree is an array of the node

    if (this->count == MAX_TABLE){
        std::cout << "error: Full Table" << std::endl;
        return;
    }

    node *new_node = new node;

    new_node->location = x_new;
    new_node->rand = x_rand;
    new_node->idx_parent = idx_near;
    new_node->alpha = alpha;
    new_node->d = d;
    new_node->idx = this->count;	
    this->count++;

    ptrTable[new_node->idx] = new_node;
    
}

int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    //TODO: check if its okay

    //this->generator.seed(time(NULL));
    // random init
    // unsigned int seed = static_cast<unsigned int>(std::time(0));
    // unsigned int seed = 1605966912;
    // std::srand(seed);
    std::srand(std::time(NULL));
    // printf("RANDOM SEED: %u\n", seed);
    
    

    double dist_thresh = 0.2;				// tune this parameter
    int it = 0;						// iteration : minimum K
    double closest_dist = distance(this->x_init, this->x_goal);	// current closest distance from a node to goal

    //while ( it < K || closest_dist > dist_thresh ){
    for (it = 0; it < K; it ++){

        // step 1
        point x_rand;

        if (it % 5 ==0){
            x_rand = this->x_goal;
        }
        else{
            x_rand = this->randomState(x_max, x_min, y_max, y_min);
        }

        // step 2
        int idx_near = this->nearestNeighbor(x_rand, MaxStep);
        if (idx_near < 0 || ptrTable[idx_near] == NULL){
            continue;
        }

        point x_near = ptrTable[idx_near]->location;
        // step 3
        double out[5];
        bool invalid = this->randompath(out, x_near, x_rand, MaxStep);
        if (invalid){
            continue;
        }
        point x_new = {out[0], out[1], out[2]};

        // step 4
        this->addVertex(x_new, x_rand, idx_near, out[3], out[4]);
        //it++;

        // update loop conditions
        if (distance(x_new, this->x_goal) < closest_dist)
            closest_dist = distance(x_new, x_goal);
        
        if (closest_dist < dist_thresh){
            break;
        }

        // if (it % 200){
        //     printf("%d th iteration for generateRRT\n", it);
        // }

    }

    if (it == K){
        printf("Sadly...\n");
        this->visualizeTree();
        return 0;
    }

    return 1;

}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    
    // randomly sample a point in a real world

    //TODO Check if its okay

    point x_rand;

    x_rand.x = random_gen(x_min, x_max);
    x_rand.y = random_gen(y_min, y_max);
    // x_rand.th = atan2(x_rand.y, x_rand.x); // ok?
    x_rand.th = random_gen(-PI, PI); // ok??????

    return x_rand;

}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {

    // find the closest point among existing nodes to x_rand
    // its different with nearestNeighbor(point x_rand)
    // you should consider the difference of the angle!
    // --> new constraint: theta < theta_max
    // TODO: How to get theta_max from MaxStep?
    

    //TODO
    int min_scale = 8; // tune this parameter for minimum step ssize
    // double R_threshold = 0.5;

    // x_rand: x_rand.x, x_rand.y, x_rand.th
    // double alpha = random_gen(0, max_alpha);
    // double d = random_gen(MaxStep/min_scale, MaxStep);
    double R = L / tan(max_alpha); // have to change radius!
    double max_beta = MaxStep / R;

    // double theta_max = x_rand.th + max_beta;

    // ptrTable: int idx, point rand, point location, int idx_parent, double alpha, double d

    double min_dist = distance(x_rand, ptrTable[0]->location);
    int min_idx = 0;

    // not sure!
    // for (int i = 1; i < this->count; i++){
    //     if (this->ptrTable[i] == NULL){
    //         continue;
    //     }

    //     double dist = distance(x_rand, ptrTable[i]->location);
    //     double radius_itr = L/tan(ptrTable[i]->alpha);

    //     double theta_itr = (ptrTable[i]->d)/ (L/tan(ptrTable[i]->alpha)) + ptrTable[i]->location.th;
    //     if ((dist < min_dist) && (theta_itr >= x_rand.th - max_beta) && (theta_itr <= x_rand.th + max_beta)){
    //         min_dist = dist;
    //         min_idx = i;
    //     }
    //     // if ((dist<min_dist) && abs(radius_itr) > R_threshold){
    //     //     min_dist = dist;
    //     //     min_idx = i;
    //     // }
    // }

    for (int i = 1; i < this->count; i++){

        if (this->ptrTable[i] == NULL)
            continue;

        point tmp = { ptrTable[i]->location.x, ptrTable[i]->location.y, ptrTable[i]->location.th };
        double x_c1 = tmp.x + R * sin(tmp.th);
        double y_c1 = tmp.y - R * cos(tmp.th);
        double x_c2 = tmp.x - R * sin(tmp.th);
        double y_c2 = tmp.y + R * sin(tmp.th);

        point c1 = {x_c1, y_c1, 0};
        point c2 = {x_c2, y_c2, 0};

        if (distance(x_rand, c1) < R && distance(x_rand, c2) < R)
            continue;

        double dist = distance(x_rand, ptrTable[i]->location);
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }

    return min_idx;

}

// Done
int rrtTree::nearestNeighbor(point x_rand) {

    // find the closest point among existing nodes to x_rand
    // just select the node with the shortest linear distance to x_rand
    // return the index of the closest node

    int min_idx;
    double min_dist;

    // Compute dist with first node
    min_dist = distance(x_rand, ptrTable[0]->location);
    min_idx = 0;

    // iterate over all nodes and find min idx
    for (int i = 1; i < this->count; i++){
        if (this->ptrTable[i] == NULL){
            continue;
        }

        double dist = distance(x_rand, ptrTable[i]->location);
        if (dist < min_dist){
            min_dist = dist;
            min_idx = i;
        }
    }

    return min_idx;

}

int rrtTree::randompath(double *out, point x_near, point x_rand, double MaxStep) {

    //TODO

    int min_scale = 8; // tune this parameter for minimum step ssize
    int num_path = 50; // tune this parameter for number of sampling
    traj paths[50]; // size should be num_path

    double alpha, d; //alpha, d of closest path

    //double d = INT_MAX; 


    // first, create some random paths starting from x_near
    for (int i = 0; i < num_path; i++){
        paths[i].x = x_rand.x;
        paths[i].y = x_rand.y;
        paths[i].th = x_rand.th;
        //paths[i].d = random_gen(MaxStep/min_scale, MaxStep);
        paths[i].d = random_gen(0.5*MaxStep, MaxStep);
        paths[i].alpha = random_gen((-1) * max_alpha, max_alpha);
    }


    // second, choose the path closest to x_rand(set x_new and alpha, d)
    int min_idx = -1;
    double min_dist = -1;

    for (int i = 0; i < num_path; i++){
        point x_tmp; // x_new candidate
        double R = L/tan(paths[i].alpha);
        double x_c = x_near.x - R * sin(x_near.th);
        double y_c = x_near.y + R * cos(x_near.th);
        double beta = paths[i].d/R;
        x_tmp.th = x_near.th + beta;
        x_tmp.x = x_c + R * sin(x_tmp.th);
        x_tmp.y = y_c - R * cos(x_tmp.th);
        paths[i].x = x_tmp.x;
        paths[i].y = x_tmp.y;
        paths[i].th = x_tmp.th;

      if (i == 0) {
          min_idx = i;
          min_dist = distance(x_rand, x_tmp);
      }
      else {
        if (distance(x_rand, x_tmp) < min_dist) {
          min_idx = i;
          min_dist = distance(x_rand, x_tmp);
        }
      }
    }

    point x_new;
    x_new.x = paths[min_idx].x;
    x_new.y = paths[min_idx].y;
    x_new.th = paths[min_idx].th;
    alpha = paths[min_idx].alpha;
    d = paths[min_idx].d;

    
    // third, set out[] and check collision of chosen path
    out[0] = x_new.x;
    out[1] = x_new.y;
    out[2] = x_new.th;
    out[3] = alpha;
    out[4] = d;
    return int(isCollision(x_near, x_new, d, L/tan(alpha)));
    
}

bool rrtTree::isCollision(point x1, point x2, double d, double R) {	

    // whether path x1->x2 (not straight line, but 'path') crosses the obstacle
    // refer to page5(pdf) for the names of variables

    // bool direction = true;
    // if (R < 0){
    //     direction = false;
    // }

    // R = abs(R);

    double x_c = x1.x - R * sin(x1.th);
    double y_c = x1.y + R * cos(x1.th);

    double th_temp = x1.th;
    int iter_bound = int(d / this->res) + 1;

    for(int i = 0; i < iter_bound; i++){	// point x2 not considered - need to check x2?
      double beta = i * (this->res) / R;

      if (i == iter_bound - 1){
          beta = d / R;
      }

    //   if (!direction){
    //       beta = - beta;
    //   }

      double x_n = x_c + R * sin(th_temp + beta);
      double y_n = y_c - R * cos(th_temp + beta);

      int x_map = round(x_n / this->res + this->map_origin_x);
      int y_map = round(y_n / this->res + this->map_origin_y);

      //if (this->map.at<uchar>(int((x_n/(this->res)) + (this->map_origin_x)), int((y_n/(this->res)) + (this->map_origin_y))) == 0 ){
      //  return true;
      //}

    //   if (x_map < 0 || x_map > round(this->map_origin_x * 2 + 0.5)){
    //       return true;
    //   }
    //   else if (y_map < 0 || y_map > round(this->map_origin_y * 2 + 0.5)){
    //       return true;
    //   }
      if (map.at<uchar>(x_map, y_map) <= 125){
          return true;
      }

    }

    int x_map = round(x2.x / this->res + this->map_origin_x);
    int y_map = round(x2.y / this->res + this->map_origin_y);



    // if (x_map < 0 || x_map > round(this->map_origin_x * 2 + 0.5)){
    //     return true;
    // }
    // else if (y_map < 0 || y_map > round(this->map_origin_y * 2 + 0.5)){
    //     return true;
    // }
    if (map.at<uchar>(x_map, y_map) <= 125){
        return true;
    }

    return false;
    // memo: unknown region not considered
}

// std::vector<traj> rrtTree::backtracking_traj(){
//     //TODO
//     std::vector<traj> result;

//     traj tmp_goal;
// 	tmp_goal.x = this->x_goal.x;
// 	tmp_goal.y = this->x_goal.y;
// 	tmp_goal.th = this->x_goal.th;
// 	tmp_goal.alpha = 0;
// 	tmp_goal.d = 0;
// 	result.push_back(tmp_goal);
    
//     int curr_idx = this->nearestNeighbor(this->x_goal);
//     point t;
//     t.x = tmp_goal.x; t.y = tmp_goal.y; t.th = tmp_goal.th;
//     point leaf;
//     leaf.x = this->ptrTable[curr_idx]->location.x;
//     leaf.y = this->ptrTable[curr_idx]->location.y;
//     leaf.th = this->ptrTable[curr_idx]->location.th;
//     printf("distance between leaf and waypoint: %f\n", distance(leaf, t));
//     while (curr_idx > 0){
// 	// temporary trajectory
//         traj tmp_t;
//         tmp_t.x = this->ptrTable[curr_idx]->location.x;
//         tmp_t.y = this->ptrTable[curr_idx]->location.y;
//         tmp_t.th = this->ptrTable[curr_idx]->location.th;
//         tmp_t.alpha = this->ptrTable[curr_idx]->alpha;
//         tmp_t.d = this->ptrTable[curr_idx]->d;
//         result.push_back(tmp_t);

//         if (curr_idx == 0){
//             break;
//         }

//         curr_idx = this->ptrTable[curr_idx]->idx_parent;	// should choose among parents, but not considered yet!!!!
//         // printf("curr_idx: %d\n", curr_idx);
//     }
    
    

//     // for (int i=0;i<result.size();i++)
//         // printf("result: (%f, %f)\n", result[i].x, result[i].y);
//     //add root to path
//     // printf("current index: %d\n", curr_idx);
//     // tmp_t.x = this->ptrTable[curr_idx]->location.x;
// 	// tmp_t.y = this->ptrTable[curr_idx]->location.y;
// 	// tmp_t.th = this->ptrTable[curr_idx]->location.th;
// 	// tmp_t.alpha = this->ptrTable[curr_idx]->alpha;
// 	// tmp_t.d = this->ptrTable[curr_idx]->d;
    
//     return result;	// doesnt contain root currently, but should it?
    
// }
std::vector<traj> rrtTree::backtracking_traj(){
    //TODO
    std::vector<traj> result;

    // traj tmp_goal;
	// tmp_goal.x = this->x_goal.x;
	// tmp_goal.y = this->x_goal.y;
	// tmp_goal.th = this->x_goal.th;
	// tmp_goal.alpha = 0;
	// tmp_goal.d = 0;
	// //result.push_back(tmp_goal);
    
    int curr_idx = this->nearestNeighbor(this->x_goal);
    // point t;
    // t.x = tmp_goal.x; t.y = tmp_goal.y; t.th = tmp_goal.th;
    point leaf = { this->ptrTable[curr_idx]->location.x, this->ptrTable[curr_idx]->location.y, this->ptrTable[curr_idx]->location.th };
    // printf("distance between leaf and waypoint: %f\n", distance(leaf, t));
    int it = 0;
    while (curr_idx >= 0){
	// temporary trajectory
        //printf("curr_idx: %d\n", curr_idx);
        leaf = { this->ptrTable[curr_idx]->location.x, this->ptrTable[curr_idx]->location.y, this->ptrTable[curr_idx]->location.th };
        traj tmp_t;
        tmp_t.x = leaf.x;
        tmp_t.y = leaf.y;
        tmp_t.th = leaf.th;
        tmp_t.alpha = this->ptrTable[curr_idx]->alpha;
        tmp_t.d = this->ptrTable[curr_idx]->d;
        
        result.push_back(tmp_t);

        if(curr_idx == 0)
            break;
        
        curr_idx = this->ptrTable[curr_idx]->idx_parent;	// should choose among parents, but not considered yet!!!!
        // printf("curr_idx: %d\n", curr_idx);

    }
    
    

    // for (int i=0;i<result.size();i++)
        // printf("result: (%f, %f)\n", result[i].x, result[i].y);
    //add root to path
    // printf("current index: %d\n", curr_idx);
    // tmp_t.x = this->ptrTable[curr_idx]->location.x;
	// tmp_t.y = this->ptrTable[curr_idx]->location.y;
	// tmp_t.th = this->ptrTable[curr_idx]->location.th;
	// tmp_t.alpha = this->ptrTable[curr_idx]->alpha;
	// tmp_t.d = this->ptrTable[curr_idx]->d;
    
    return result;	// doesnt contain root currently, but should it?
    
}

double distance(point p1, point p2){

    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

}

double random_gen(double min_val, double max_val){
    // double result = min_val + static_cast <double> (std::rand()) /( static_cast <double> (RAND_MAX/(max_val - min_val)));
    // while( result == 0 )
    //     result = min_val + static_cast <double> (std::rand()) /( static_cast <double> (RAND_MAX/(max_val - min_val)));
    // return result;  
    double result = min_val + double(std::rand())/RAND_MAX * (max_val - min_val);
    while( result == 0 )
        result = min_val + double(std::rand())/RAND_MAX * (max_val - min_val);
    return result;   
}