/*
 * target_shift.cpp
 * 
 *  Created on: Aug 14th, 2020
 *      Author: Hilbert Xu
 *   Institute: Mustar Robot
 *   使用前修改头部电机速度为0.1
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
#include <boost/thread/shared_mutex.hpp>

// OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
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

typedef message_filters::sync_policies::ApproximateTime<dynamixel_msgs::JointState, dynamixel_msgs::JointState, robot_vision_msgs::BoundingBoxes> syncPolicy;

float calcPixelDistance(int targetX_, int targetY_, int X_, int Y_) {
  float distance = sqrt(pow((targetX_ - X_), 2) + pow((targetY_ - Y_),2));
  return distance;
}

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
private:
  // Image parameters
  int centerX_;
  int centerY_;
  int currX_ = 0;
  int currY_ = 0;
  int preCurrX_ = 0;
  int preCurrY_ = 0;
  int frameCount_  = 0;
  int innerThreshold_ = 50;
  float currPanJointState_;
  float currLiftJointState_;

  // Flags
  bool FLAG_start_track = false;

  // ROS node Handle
  ros::NodeHandle nodeHandle_;

  // ROS publishers & subscribers
  std::string targetName_;

  ros::Publisher headPanJointPublisher_;
  ros::Publisher headLiftJointPublisher_;

  ros::Subscriber cameraInfoSubscriber_;
  ros::Subscriber controlSubscriber_;
public:
  // 构造函数和析构函数
  TargetShift(ros::NodeHandle nh, int argc, char** argv):nodeHandle_(nh){
    if (argc == 3 && std::string(argv[1])=="-target") {
      targetName_ = std::string(argv[2]);
      ROS_INFO("[TargetShift] Receivied Track target: %s from console", targetName_.c_str());
      FLAG_start_track = true;
      init(argc, argv);
    } else {
      init(argc, argv);
    }
  }

  ~TargetShift() {}

  void dynamixelControl(int curr_x, int curr_y, float pan_state, float lift_state, float scale) {
    int error_x = centerX_ - curr_x;
    int error_y = centerY_ - curr_y;
    printf("X error: %d, Y error: %d", error_x, error_y);

    float next_state_pan =  pan_state + error_x*scale;
    float next_state_lift = lift_state - error_y*scale;
    std_msgs::Float64 msg_pan;
    std_msgs::Float64 msg_lift;

    msg_pan.data = next_state_pan;
    msg_lift.data = next_state_lift;

    headPanJointPublisher_.publish(msg_pan);
    headLiftJointPublisher_.publish(msg_lift);
  }

  void controlCallback(robot_control_msgs::Mission msg) {
    if (msg.action == "track") {
      if (msg.target == "object") {
        assert(msg.attributes.object.name != "");
        targetName_ = msg.attributes.object.name; 
        FLAG_start_track = true;
      }
      if (msg.target == "human") {
        targetName_ = "person";
        FLAG_start_track = true;
      }
    }
    ROS_INFO("[TargetShift] Receivied Track target: %s from control node", targetName_.c_str());
  }

  // Camera Info callback
  void cameraInfoCallback(sensor_msgs::CameraInfo msg) {
    // 读取一次相机信息，获得图像帧的中心点
    if (centerX_==0 && centerY_==0){
      centerX_ = int(msg.width/2);
      centerY_ = int(msg.height/2);
      ROS_INFO("[CameraInfo] Setting pixel_center_x: %d, pixel_center_y:%d...", centerX_, centerY_);
    }
  }

  void currStateCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const robot_vision_msgs::BoundingBoxesConstPtr &msg) {
    if (FLAG_start_track) {
      std::cout << "======================= Enter Sync callback ===========================" << std::endl;
      // searching for target object
      for (int i = 0; i < msg->bounding_boxes.size(); i++) {
        if (msg->bounding_boxes[i].Class == targetName_) {
          // 保留上一帧中的识别框中心点
          preCurrX_ = currX_;
          preCurrY_ = currX_;
          // 记录当前识别框的中心点
          currX_ = int((msg->bounding_boxes[i].xmax + msg->bounding_boxes[i].xmin) / 2);
          currY_ = int((msg->bounding_boxes[i].ymax + msg->bounding_boxes[i].ymin) / 2);
          printf("Pre frame: (%d, %d), current frame: (%d, %d)\n", preCurrX_, preCurrY_, currX_, currY_);
          // 如果识别到多个目标物体怎么办。。。
          // 目前尝试方案：识别到目标物体后直接break出当前循环
          break;
        }
      }
      // receive motor state
      currPanJointState_ = pan_state->current_pos;
      currLiftJointState_ = lift_state->current_pos;
    
      dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
    }
  }

  void init(int argc, char** argv) {
    ROS_INFO("[TargetShift] Initializing...");
    // Initialize ROS publishers & subscribers
    headPanJointPublisher_  = nodeHandle_.advertise<std_msgs::Float64>("/head_pan_joint/command", 1, false);
    headLiftJointPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("/head_lift_joint/command", 1, false);

    cameraInfoSubscriber_   = nodeHandle_.subscribe("/astra/rgb/camera_info", 1, &TargetShift::cameraInfoCallback, this);
    controlSubscriber_      = nodeHandle_.subscribe("/control_to_vision", 1, &TargetShift::controlCallback, this);

    // 回调函数同步，确保获取同一时刻两个头部电机的关节角度和识别框的位置
    message_filters::Subscriber<dynamixel_msgs::JointState> panJointSubscriber_(nodeHandle_, "/head_pan_joint/state", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<dynamixel_msgs::JointState> liftJointSubscriber_(nodeHandle_, "/head_lift_joint/state", 1, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<robot_vision_msgs::BoundingBoxes> bboxSubscriber_(nodeHandle_, "/yolo_ros/bounding_boxes", 1, ros::TransportHints().tcpNoDelay());
    
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), panJointSubscriber_, liftJointSubscriber_, bboxSubscriber_);
    sync.registerCallback(boost::bind(&TargetShift::currStateCallback, this, _1, _2, _3));
    ros::spin();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "track_target");
  ROS_INFO("[TargetShift] Initializing...");
  ros::NodeHandle nh("~");
  TargetShift tracker(nh, argc, argv);  
  return 0;
}