/*
 * target_shift.cpp
 * 
 *  Created on: Aug 14th, 2020
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

typedef message_filters::sync_policies::ApproximateTime<dynamixel_msgs::JointState, dynamixel_msgs::JointState> syncPolicy;

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
  // ROS Handle
  ros::NodeHandle nodeHandle_;

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

  // ROS publishers & subscribers
  std::string targetName_;

  ros::Publisher headPanJointPublisher_;
  ros::Publisher headLiftJointPublisher_;

  ros::Subscriber cameraInfoSubscriber_;
  ros::Subscriber bboxSubscriber_;
  ros::Subscriber controlSubscriber_;
public:
  // 构造函数和析构函数
  TargetShift(ros::NodeHandle nh):nodeHandle_(nh) {}
  
  TargetShift(ros::NodeHandle nh, std::string target):nodeHandle_(nh), targetName_(target) {
    ROS_INFO("[TargetShift] Receivied Track target: %s from console", targetName_.c_str());
    FLAG_start_track = true;
    init();
  }

  ~TargetShift() {}

  void pubPanJointCommand(ros::Publisher &publisher, int error, float scale) {
    std_msgs::Float64 msg;
    
  }

  //! 用来控制头部电机大致朝向物体的函数
  // void controlHead(int X_, int Y_, float controlScale_) {
  //   {
  //     // head lift control
  //     float error = centerY_ - Y_;
  //     float pan = currPanJointState_ + error*controlScale_;
  //   }
  //   {
  //     // head pan control
  //   }

  // }

  //! 用来在物体静止后控制电机精确朝向物体的函数
  // int pidController(int X_, int Y_) {
  //   // @TODO 
  //   // 目前使用简单的对准策略，考虑使用上面实现的增量pid进行控制
  //   float error = calcPixelDistance(centerX_, centerY_, X_, Y_);
  //   if (error < 10) {
  //     return 0;
  //   }
  //   if (error > 10) {
  //     return pidController(currX_, currY_);
  //   }
  // }

  void controlCallback(robot_control_msgs::Mission msg) {
    if (msg.action == "track") {
      if (msg.target == "object") {
        assert(msg.attributes.object.name != "");
        targetName_ = msg.attributes.object.name; 
        FLAG_start_track = true;
        init();
      }
      if (msg.target == "human") {
        targetName_ = "person";
        FLAG_start_track = true;
        init();
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
      ROS_INFO("[CameraInfo] Setting centerX_: %d, centerY_:%d...", centerX_, centerY_);
    }
  }

  // Bounding boxes callback
  void bboxCallback(robot_vision_msgs::BoundingBoxes msg) {
    // searching for target object
    for (int i = 0; i < msg.bounding_boxes.size(); i++) {
      if (msg.bounding_boxes[i].Class == targetName_) {
        // 保留上一帧中的识别框中心点
        preCurrX_ = currX_;
        preCurrY_ = currX_;
        // 记录当前识别框的中心点
        currX_ = int((msg.bounding_boxes[i].xmax + msg.bounding_boxes[i].xmin) / 2);
        currY_ = int((msg.bounding_boxes[i].ymax + msg.bounding_boxes[i].ymin) / 2);
        printf("Pre frame: (%d, %d), current frame: (%d, %d)", preCurrX_, preCurrY_, currX_, currY_);
        // 如果识别到多个目标物体怎么办。。。
        // 目前尝试方案：识别到目标物体后直接break出当前循环
        break;
      }
    }
  }

  void jointStateCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state) {
    currPanJointState_ = pan_state->current_pos;
    currLiftJointState_ = lift_state->current_pos;
  }

  void init() {
    ROS_INFO("[TargetShift] Initializing...");
    // Initialize ROS publishers & subscribers
    headPanJointPublisher_  = nodeHandle_.advertise<std_msgs::Float64>("/head_pan_joint/command", 1, false);
    headLiftJointPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("/head_lift_joint/command", 1, false);

    cameraInfoSubscriber_   = nodeHandle_.subscribe("/astra/rgb/camera_info", 1, &TargetShift::cameraInfoCallback, this);
    controlSubscriber_      = nodeHandle_.subscribe("/control_to_vision", 1, &TargetShift::controlCallback, this);
    bboxSubscriber_         = nodeHandle_.subscribe("/yolo_ros/bounding_boxes", 1, &TargetShift::bboxCallback, this);

    // 回调函数同步，确保获取同一时刻两个头部电机的关节角度
    message_filters::Subscriber<dynamixel_msgs::JointState> panJointSubscriber_(nodeHandle_, "/head_pan_joint/state", 10, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<dynamixel_msgs::JointState> liftJointSubscriber_(nodeHandle_, "/head_lift_joint/state", 10, ros::TransportHints().tcpNoDelay());
    
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), panJointSubscriber_, liftJointSubscriber_);
    sync.registerCallback(boost::bind(&TargetShift::jointStateCallback, this, _1, _2));
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "track_target");
  ros::NodeHandle nh;

  if (argc == 3 && std::string(argv[1]) == "-target") {
    // 如果控制台中有参数track_target，则将参数值作为追踪目标
    TargetShift tracker(nh, std::string(argv[2]));
  } else {
    TargetShift tracker(nh);
  }

  ros::spin();
  
}