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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <dynamixel_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Robot control msgs
#include <robot_control_msgs/Mission.h>
#include <robot_control_msgs/Results.h>
#include <robot_control_msgs/Feedback.h>
#include <robot_vision_msgs/BoundingBoxes.h>
#include <robot_vision_msgs/HumanPoses.h>

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
  int currX_;
  int currY_;
  int preCurrX_ = 0;
  int preCurrY_ = 0;
  int frameCount_  = 0;
  int innerThreshold_ = 50;
  float currPanJointState_;
  float currLiftJointState_;

  // Flags
  bool FLAG_start_track = false;
  bool FLAG_coarse_tune_ = false;
  bool FLAG_fine_tune_ = false;
  bool FLAG_request_curr_state = true;

  // ROS publishers & subscribers
  std::string targetNames_;
  ros::Publisher headTrajectoryPublisher_;
  ros::Subscriber cameraInfoSubscriber_;
  ros::Subscriber bboxSubscriber_;
  ros::Subscriber controlSubscriber_;

  // Sync subscribers
  ros::Subscriber panJointSubscriber_;
  ros::Subscriber liftJointSubscriber_;
public:
  // 构造函数和析构函数
  TargetShift() {};
  ~TargetShift();

  //! 用来控制头部电机大致朝向物体的函数
  void controlHead(int X_, int Y_, float controlScale_);

  //! 用来在物体静止后控制电机精确朝向物体的函数
  int pidController(int X_, int Y_) {
    float error = calcPixelDistance(centerX_, centerY_, X_, Y_);
    if (error < 5) {
      return 0;
    }
    if (error > 5) {
      controlHead(X_, Y_, 0.001);
      return pidController(currX_, currY_);
    }
  }

  void controlCallback(robot_control_msgs::Mission msg) {
    ROS_INFO("TO BE DONE...");
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

  void jointsCallback(dynamixel_msgs::JointState pan_joint, dynamixel_msgs::JointState lift_joint) {
    // 在需要获取当前joint状态时，读取一次当前的joint状态
    if (FLAG_request_curr_state) {
      currPanJointState_  = pan_joint.current_pos;
      currLiftJointState_ = lift_joint.current_pos;
      FLAG_request_curr_state = false;
    }
  }

  // Bounding boxes callback
  void bboxCallback(robot_vision_msgs::BoundingBoxes msg) {
    
    // searching for target object
    for (int i = 0; i < msg.bounding_boxes.size(); i++) {
      if (msg.bounding_boxes[i].Class == targetNames_) {
        currX_ = int((msg.bounding_boxes[i].xmax + msg.bounding_boxes[i].xmin) / 2);
        currY_ = int((msg.bounding_boxes[i].ymax + msg.bounding_boxes[i].ymin) / 2);
        // 如果识别到多个目标物体怎么办。。。
        // 目前尝试方案：识别到目标物体后直接break出当前循环
        break;
      }
    }
    // @TODO 改写称为多线程
    // Thread 1: 读取bbox
    // Thread 2: 根据currX_, currY_控制头部摄像机
    if (!FLAG_fine_tune_) {
      // 如果已经进入精调阶段，则跳过以下判断，该回调函数仅更新物体当前时刻的识别框中心点
      if (frameCount_ < 10) {
        // 计算当前识别框与图像中心点的距离
        float centerDistance = calcPixelDistance(currX_, currY_, centerX_, centerY_);
        if (centerDistance > innerThreshold_) {
          // 认为之前的识别帧累计无效，清零
          frameCount_ = 0;
          // FLAG:粗调
          FLAG_coarse_tune_ = true;
          // 对摄像机进行一个粗调，朝向物体大致位置
          controlHead(currX_, currY_, 0.001);
        } else {
          // 当前识别帧与图像中心相差小于50
          // 计算两帧识别框之间的距离
          float preDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
          if (preDistance < 5) {
            // 如果两帧识别框距离小于5，则保持不动
            frameCount_ += 1;
          } else {
            frameCount_ = 0;
            // FLAG:粗调
            FLAG_coarse_tune_ = true;
            controlHead(currX_, currY_, 0.0005);
          }
        }
      } else {
        // 连续10帧检测到物体中心点移动不超过5个像素点，认为物体当前处于静止状态，利用一个PID控制器控制相机精确朝向物体
        FLAG_fine_tune_ = true;
        pidController(currX_, currY_);
        }
    }
    
      
    }
  }
};