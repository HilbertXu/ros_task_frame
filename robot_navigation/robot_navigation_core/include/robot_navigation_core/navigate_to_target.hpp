/*
 * navigate_to_target.hpp
 * 
 *  Created on: 28th Aug 2020
 *      Author: Hilbert Xu
 *   Institute: Mustar Robot
 */

// c++
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <time.h>
#include <pthread.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <sys/time.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>

//move_base头文件
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
//actionlib头文件
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace navigate_to_target {
  class NavigateToTarget {
  public:
    NavigateToTarget (ros::NodeHandle nh, int argc, char** argv);
    ~NavigateToTarget(); 

  protected:
    ros::NodeHandle nodeHandle_;
    geometry_msgs::Pose targetPose_;
    nav_msgs::OccupancyGrid map_;

    // ROS publishers & subscribers
    ros::Subscriber mapSubscriber_;
    ros::Subscriber targetPoseSubscriber_;

    // Flags
    bool FLAG_start_navigate = false;

    // Check if current target is valid
    void checkValid(geometry_msgs::Pose worldPose);

    // Pixel coordinates to world coordiantes
    std::vector<int> mapToWorld(int map_x, int map_y);

    // World coordinates to pixel coordinates
    std::vector<int> worldToMap(float world_x, float world_y);

    // Find nearest valid point in map if needed
    int findNearestValid(int index);

    // Subscribe map
    void mapCallback(const nav_msgs::OccupancyGridConstPtr &msg);

    // callback of target pose
    void targetPoseCallback(const geometry_msgs::PoseConstPtr &msg);
  }
}