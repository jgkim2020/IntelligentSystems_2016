
//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include <project4/purePursuit.h>
#include <project4/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pwd.h>
#include <ctime>
#include <iostream>
#include <cstdio>

//map spec
cv::Mat map;
cv::Mat dynamic_map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;
int map_update_cnt = 0;

//way points
std::vector<point> waypoints;

//path
std::vector<point> path_RRT;

//robot
point robot_pose;
geometry_msgs::Twist cmd_vel;

//point cloud data from kinect
pcl::PointCloud<pcl::PointXYZ> point_cloud;

//FSM state
int state = INIT;

int look_ahead_idx;
int waypoints_idx;
bool state_turn = false;

//function definition
bool isCollision();
void dynamic_mapping(ros::Publisher pub);
void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msgs );
void set_waypoints();
int generate_path_RRT();
void callback_points(sensor_msgs::PointCloud2ConstPtr msgs);
void setcmdvel(double v, double w);

int main(int argc, char** argv){

    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("server_msg",15,poseCallback);
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",100);
    ros::Subscriber gazebo_camera_sub = n.subscribe("/camera/depth/points",100, callback_points);
    printf("Initialize topics\n");

    // Load Map
    char* user = getlogin();
    map = cv::imread((std::string("/home/")+
                      std::string(user)+
                      std::string("/catkin_ws/src/project4/src/contest2016.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    map_origin_x = 250.0 - 0.5;
    map_origin_y = 100.0 - 0.5;
    world_x_min = -2500.0;
    world_x_max = 2500.0;
    world_y_min = -1000.0;
    world_y_max = 3500.0;
    res = 10;
    printf("Load map\n");

    dynamic_map = map.clone();

    // Set Way Points

    // FSM
    state = INIT;
    bool running = true;
    purePursuit pure_pursuit;
    ros::Rate control_rate(10);
    ros::spinOnce();
    control_rate.sleep();
    control_rate.sleep();
    control_rate.sleep();
    ros::spinOnce();
    control_rate.sleep();
    control_rate.sleep();
    control_rate.sleep();

    point starting_point;
    starting_point.x = robot_pose.x;
    starting_point.y = robot_pose.y;
    waypoints.push_back(starting_point);
    set_waypoints();
    printf("Set way points\n");

    if(generate_path_RRT() == 0) printf("Generate RRT success!\n");
    else printf("Generate RRT fail!\n");
    while(running){
      switch (state) {
      case INIT: {
          look_ahead_idx = 0;
          waypoints_idx = 1;

          ros::spinOnce();
          ros::Rate(0.33).sleep();
          printf("Initialize ROBOT\n");

          state = RUNNING;
      } break;

      case RUNNING: {
          //TODO
          /*
           * copy your code from previous project2
           */

          /*
           * add transition part from RUNNING to PATH_PLANNING
           * when meeting obstacle
           */
          control ctrl = pure_pursuit.get_control(robot_pose, path_RRT[look_ahead_idx]);
          double distance = pow(robot_pose.x - path_RRT[look_ahead_idx].x, 2) + pow(robot_pose.y - path_RRT[look_ahead_idx].y, 2);
          if(ctrl.v == 0 || state_turn == true) { // additional control for turning
              state_turn = true;
              ctrl.v = 0;
              double angle = ctrl.w;
              if(angle > 1) ctrl.w = 0.3; // upper bound
              else if(angle < -1) ctrl.w = -0.3; // lower bound
              else if(angle < 0.05 && angle > -0.05) { // stop turning within certain threshold
                  ctrl.w = 0;
                  state_turn = false;
              }
              else {
                  if(angle > 0) ctrl.w = 0.2*angle + 0.1; // ctrl.w: 0.1~0.5
                  else ctrl.w = 0.2*angle - 0.1; // ctrl.w: -0.5~-0.1
              }
          }
          setcmdvel(ctrl.v, ctrl.w);
          cmd_vel_pub.publish(cmd_vel);

      /*    if(isCollision()) {
              printf("\nUnexpected obstacle has emerged!\n\n");
              state = PATH_PLANNING;
              setcmdvel(0,0);
              cmd_vel_pub.publish(cmd_vel);
          }*/
          if(distance < 200*200) {
              if(path_RRT[look_ahead_idx].x == waypoints[waypoints_idx].x && path_RRT[look_ahead_idx].y == waypoints[waypoints_idx].y) waypoints_idx++;
              look_ahead_idx++;
              if(waypoints_idx == waypoints.size()) state = FINISH;
          }


          ros::spinOnce();
          control_rate.sleep();
      } break;

      case PATH_PLANNING: {
          //TODO
          /*
           * do dynamic mapping from kinect data
           * pop up the opencv window
           * after drawing the dynamic map, transite the state to RUNNING state
           */
          if(map_update_cnt%5 == 0) dynamic_map = map.clone(); //VARIABLEL
          map_update_cnt++;
          dynamic_mapping(cmd_vel_pub);
          printf("\nUpdated map\n");
          /*cv::namedWindow("Dynamic Mapping");
          cv::imshow("Dynamic Mapping", dynamic_map);
          cv::waitKey(5000000);*/
          if(generate_path_RRT() == 0) {
              printf("Path replanning success!\n\n");
              state = RUNNING;
          }
          else {
              printf("Path replanning fail! Clearing dynamic map\n");
              dynamic_map = map.clone();
          }

          ros::spinOnce();
          control_rate.sleep();
      } break;

      case FINISH: {
          setcmdvel(0,0);
          cmd_vel_pub.publish(cmd_vel);
          running = false;

          ros::spinOnce();
          control_rate.sleep();
      } break;

      default: {
      } break;
      }

      printf("curr state : %d\ncurr waypoint : %d\ncurr robot pos : %.2f,%.2f\ncurr robot vel : %.2f,%.2f\n",state,waypoints_idx,robot_pose.x,robot_pose.y,cmd_vel.linear.x,cmd_vel.angular.z);
  }

    return 0;
}

int generate_path_RRT()
{
    //TODO
    /*
     * copy your code from previous project2
     */
    int result = 0;
    if (state == INIT) { // execute only for the first time
        std::vector<point> path_temp;
        rrtTree *rrt_temp;
        for (int i = waypoints.size() - 1; i > 0; i--) {
            rrt_temp = new rrtTree(waypoints[i - 1], waypoints[i], map, map_origin_x, map_origin_y, res, 0.01);
            result += rrt_temp->generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 200000, 450); //VARIABLEG VARIABLEH//////////////////////////
            path_temp = rrt_temp->backtracking();
            path_RRT.insert(path_RRT.end(), path_temp.begin(), path_temp.end());
            delete rrt_temp;
        }
        std::reverse(path_RRT.begin(), path_RRT.end());
    }
    else if (state == PATH_PLANNING) { // execute during replanning
        path_RRT.clear(); // clear previous path
        point curr_pos;
        std::vector<point> waypoints_dynamic;
        std::vector<point> path_temp;
        rrtTree *rrt_temp;
        curr_pos.x = robot_pose.x;
        curr_pos.y = robot_pose.y;
        waypoints_dynamic.push_back(curr_pos); // push current postion
        for (int i = waypoints_idx; i < waypoints.size(); i++) waypoints_dynamic.push_back(waypoints[i]);
        for (int i = waypoints_dynamic.size() - 1; i > 0; i--) {
            rrt_temp = new rrtTree(waypoints_dynamic[i - 1], waypoints_dynamic[i], dynamic_map, map_origin_x, map_origin_y, res, 0.01);
            result += rrt_temp->generateRRTst(world_x_max, world_x_min, world_y_max, world_y_min, 200000, 450); //VARIABLEH VARIABLEG
            path_temp = rrt_temp->backtracking();
            path_RRT.insert(path_RRT.end(), path_temp.begin(), path_temp.end());
            delete rrt_temp;
        }
        std::reverse(path_RRT.begin(), path_RRT.end());
        look_ahead_idx = 0;
    }
    return result;
}

void set_waypoints()
{
    point waypoint_candid[3];
    waypoint_candid[0].x = -1253.0;
    waypoint_candid[0].y = 2302.0;
    waypoint_candid[1].x = 1374.0;
    waypoint_candid[1].y = 220.0;
    waypoint_candid[2].x = 1353.0;
    waypoint_candid[2].y = 2272.0;
    int order[] = {0,1,2};
    int order_size = 3;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}


void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msgs) {
    robot_pose.x = msgs->x;
    robot_pose.y = msgs->y;
    robot_pose.th = msgs->theta;
    return;
}
bool isCollision()
{
    //TODO
    /*
     * obstacle emerge in front of robot -> true
     * other wise -> false
     */
    double dist_min = 1000000000.0;
    double dist_temp;
    for(int i = point_cloud.size()/3; i < point_cloud.size()/2; i++){  //VARIABLE getting height of kinect viewer, down together
        if(!isnan(point_cloud[i].x) && !isnan(point_cloud[i].y) && !isnan(point_cloud[i].z) && fabs(point_cloud[i].x) < 250) {
            dist_temp = point_cloud[i].x * point_cloud[i].x + point_cloud[i].y * point_cloud[i].y + point_cloud[i].z * point_cloud[i].z;
            if (dist_temp < dist_min) dist_min = dist_temp;
        }
    }
    if(dist_min < 550*550) return true; //VARIABLEM
    return false;
}


void dynamic_mapping(ros::Publisher pub) // scanning without rotation
{
    //TODO
    /*
    * draw dynamic map using variable dynamic_map and variable point_cloud
    */
    std::clock_t tick_begin = std::clock();
    setcmdvel(0,0);
    pub.publish(cmd_vel);
    while((std::clock() - tick_begin)/((double)CLOCKS_PER_SEC) < 0.5); // rest for 0.5 second
    tick_begin = std::clock();
    setcmdvel(-0.1,0);
    pub.publish(cmd_vel);
    while((std::clock() - tick_begin)/((double)CLOCKS_PER_SEC) < 1); // reverse for 3 second
    tick_begin = std::clock();
    setcmdvel(0,0);
    pub.publish(cmd_vel);
    while((std::clock() - tick_begin)/((double)CLOCKS_PER_SEC) < 0.5); // rest for 0.5 second

    ros::spinOnce(); // update subscriber
    for(int i = point_cloud.size()/3; i < point_cloud.size()/2; i++){
        if(!isnan(point_cloud[i].z) && !isnan(point_cloud[i].x)) {
            int temp_x = (int)((cos(robot_pose.th)*point_cloud[i].z + sin(robot_pose.th)*point_cloud[i].x + robot_pose.x)/res + map_origin_x);
            int temp_y = (int)((sin(robot_pose.th)*point_cloud[i].z - cos(robot_pose.th)*point_cloud[i].x + robot_pose.y)/res + map_origin_y);
            for(int l = -1; l <= 1; l++) { //VARIABLEN
                for(int m = -1; m <= 1; m++) {
                    dynamic_map.at<uchar>(temp_x + l, temp_y + m) = 0;
                }
            }
        }
    }

    //cv::namedWindow("Dynamic Mapping");
    //cv::imshow("Dynamic Mapping", dynamic_map);
    //cv::waitKey(10);
}

void callback_points(sensor_msgs::PointCloud2ConstPtr msgs){
    pcl::fromROSMsg(*msgs,point_cloud);
}

void setcmdvel(double v, double w){
    cmd_vel.linear.x = v;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = w;
}
