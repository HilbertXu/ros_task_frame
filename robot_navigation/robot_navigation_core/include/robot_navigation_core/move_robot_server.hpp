/*
 * move_robot_server.hpp
 * 
 *  Created on: 21th Aug 2020
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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_navigation_msgs/MoveRobotAction.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues>

// Robot control msgs
#include <robot_control_msgs/Mission.h>
#include <robot_control_msgs/Results.h>
#include <robot_control_msgs/Feedback.h>

#define PI 3.1415926

namespace move_robot_server {
  class MoveRobotServer {
  public:
    MoveRobotServer(ros::NodeHandle nh_, std::string name);
    ~MoveRobotServer(void);
  protected:
    // ROS node handle
    ros::NodeHandle nodeHandle_;
    
    // Action server
    actionlib::SimpleActionServer<robot_navigation_msgs::MoveRobotAction> actionServer_;
    // Feedback object
    robot_navigation_msgs::MoveRobotFeedback feedBack_;
    // Result object
    robot_navigation_msgs::MoveRobotResult result_;
    bool FALG_success = true;

    // TF listener
    tf::StampedTransform transform;
    tf::TransformListener pListener;
    // target frame & source frame
    std::string odomFrame_ = "/odom";
    std::string baseFrame_ = "/base_footprint";
    
    // Parameters
    // Action name
    std::string actionName_;
    
    // target/current angle/distance
    float targetAngle_;
    float targetDistance_;
    float currAngle_;
    float currDistance_;
    
    // Angular tolerance
    float angularTolerance_;
    
    // angular/linear speed
    float linearSpeed_;
    float angularSpeed_;

    // FLAG
    bool FLAG_success = false;
    
    ros::Publisher cmdVelPublisher_;

    // Get transform from odom frame to base link frame
    tf::StampedTransform getOdomTransform() {
      pListener.lookupTransform(odomFrame_, baseFrame_, ros::Time(), transform);
      return transform;
    }

    // utilities function
    float quatToAngle(tf::Quaternion quat);

    float normalizeAngle(float angle);

    // Turn robot function
    void turnRobot(ros::Rate loop_rate, float goalRadius, tf::Quaternion rot);

    // Move robot straightly function
    void goForward(ros::Rate loop_rate, float distance, tf::Vector3 position);

    // Actionlib execute callback
    void executeCallback(const robot_navigation_msgs::MoveRobotGoalConstPtr &msg);
  };
}
