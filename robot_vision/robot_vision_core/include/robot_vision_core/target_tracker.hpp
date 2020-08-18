/*
 * target_shift.hpp
 * 
 *  Created on: Aug 14th, 2020
 *      Author: Hilbert Xu
 *   Institute: Mustar Robot
 *   使用前修改头部电机速度为0.1
 */

// #pragma once

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
#include <boost/thread/shared_mutex.hpp>

// OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv_apps/FaceArrayStamped.h>
#include <opencv_apps/RotatedRectStamped.h>
#include <dynamixel_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

//Subscribers Synchronizer 
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Robot control msgs
#include <robot_control_msgs/Mission.h>
#include <robot_control_msgs/Results.h>
#include <robot_control_msgs/Feedback.h>
#include <robot_vision_msgs/BoundingBoxes.h>
#include <robot_vision_msgs/HumanPoses.h>

// Define synchronizer policy
typedef message_filters::sync_policies::ApproximateTime<dynamixel_msgs::JointState, dynamixel_msgs::JointState, robot_vision_msgs::BoundingBoxes> YoloSyncPolicy;
typedef message_filters::Synchronizer<YoloSyncPolicy> YoloSync;

typedef message_filters::sync_policies::ApproximateTime<dynamixel_msgs::JointState, dynamixel_msgs::JointState, robot_vision_msgs::HumanPoses> OpenposeSyncPolicy;
typedef message_filters::Synchronizer<OpenposeSyncPolicy> OpenposeSync;

typedef message_filters::sync_policies::ApproximateTime<dynamixel_msgs::JointState, dynamixel_msgs::JointState, opencv_apps::RotatedRectStamped> ColorSyncPolicy;
typedef message_filters::Synchronizer<ColorSyncPolicy> ColorSync;

typedef message_filters::sync_policies::ApproximateTime<dynamixel_msgs::JointState, dynamixel_msgs::JointState, opencv_apps::FaceArrayStamped> FaceSyncPolicy;
typedef message_filters::Synchronizer<FaceSyncPolicy> FaceSync;

namespace target_tracker {
  // Distance calculate function
  float calcPixelDistance(int targetX_, int targetY_, int X_, int Y_) {
    float distance = sqrt(pow((targetX_ - X_), 2) + pow((targetY_ - Y_),2));
    return distance;
  }

  // PID class
  class IncrementalPID {
  private:
    float kp;
    float ki;
    float kd;
    float target;
    float actual;
    float e;
    float e_pre_1;
    float e_pre_2;
    float A;
    float B;
    float C;
  public:
    //增量PID
    IncrementalPID():kp(0),ki(0),kd(0),e_pre_1(0),e_pre_2(0),target(0),actual(0)
    {
      A=kp+ki+kd;
      B=-2*kd-kp;
      C=kd;
      e=target-actual;
    }
    IncrementalPID(float p, float i, float d):kp(p),ki(i),kd(d),e_pre_1(0),e_pre_2(0),target(0),actual(0)
    {
      A=kp+ki+kd;
      B=-2*kd-kp;
      C=kd;
      e=target-actual;
    }

    float pidControl(float tar,float act)
    {
      float u_increment;
      target=tar;
      actual=act;
      e=target-actual;
      u_increment=A*e+B*e_pre_1+C*e_pre_2;
      e_pre_2=e_pre_1;
      e_pre_1=e;
      return u_increment;
    }
  };

  class TargetShift {
  public:
    //! Constructor
    explicit TargetShift(ros::NodeHandle nh, int argc, char** argv);

    //! Destrcutor
    ~TargetShift();
  private:
    // Class parameters
    int centerX_ = 0;
    int centerY_ = 0;
    int currX_;
    int currY_;
    int preCurrX_;
    int preCurrY_;
    int frameCount_;
    float currPanJointState_;
    float currLiftJointState_;
    std::string track_;
    std::string targetName_;

    // FLAG
    bool FLAG_start_track = false;

    // ROS NodeHandle
    ros::NodeHandle nodeHandle_;

    // ROS subscribers & publishers
    ros::Publisher headPanJointPublisher_;
    ros::Publisher headLiftJointPublisher_;
    
    ros::Subscriber cameraInfoSubscriber_;
    ros::Subscriber controlSubscriber_;

    // Synchronizer subscribers
    message_filters::Subscriber<dynamixel_msgs::JointState> panJointSubscriber_;
    message_filters::Subscriber<dynamixel_msgs::JointState> liftJointSubscriber_;
    message_filters::Subscriber<robot_vision_msgs::BoundingBoxes> bboxSubscriber_;
    message_filters::Subscriber<robot_vision_msgs::HumanPoses> poseSubscriber_;
    message_filters::Subscriber<opencv_apps::RotatedRectStamped> colorSubscriber_;
    message_filters::Subscriber<opencv_apps::FaceArrayStamped> faceSubscriber_;

    boost::shared_ptr<YoloSync> yoloSync;
    boost::shared_ptr<OpenposeSync> openposeSync;
    boost::shared_ptr<ColorSync> colorSync;
    boost::shared_ptr<FaceSync> faceSync;

    // Initial function
    void init();

    // Set track target function
    void setTrackTarget();

    // Control dynamixel motors according to current (x,y) and current motors state
    void dynamixelControl(int curr_x, int curr_y, float pan_state, float lift_state, float scale);

    // Control callback function
    void controlCallback(const robot_control_msgs::MissionConstPtr &msg);

    // Camera info callback function
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);

    // Synchronized callback function 1. <JointState-10hz, JointState-10hz, bounding_boxes-3hz>
    void yoloTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const robot_vision_msgs::BoundingBoxesConstPtr &msg);

    // Synchronized callback function 1. <JointState-10hz, JointState-10hz, human_poses-10hz>
    void openposeTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const robot_vision_msgs::HumanPosesConstPtr &msg);

    void colorTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const opencv_apps::RotatedRectStampedConstPtr &msg);

    void faceTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const opencv_apps::FaceArrayStampedConstPtr &msg);

    void faceWithNameTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const opencv_apps::FaceArrayStampedConstPtr &msg);
  };
}