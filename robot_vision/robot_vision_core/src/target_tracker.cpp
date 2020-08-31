/*
 * target_shift.cpp
 * 
 *  Created on: Aug 14th, 2020
 *      Author: Hilbert Xu
 *   Institute: Mustar Robot
 *   使用前修改头部电机速度为0.1
 */

#include <robot_vision_core/target_tracker.hpp>

using namespace target_tracker;

TargetShift::TargetShift(ros::NodeHandle nh, int argc, char** argv): nodeHandle_(nh), actionClient_("/move_robot_server", true) {
  // 等待控制机器人移动的服务端启动
  ROS_INFO("[TargetTracker] Waiting for MoveRobotServer...");
  actionClient_.waitForServer();

  //设定节点状态参数FLAG_under_control
  if (nodeHandle_.hasParam("/under_control")) {
    // 如果rosparam服务器中存在/under_control参数
    nodeHandle_.getParam("/under_control", FLAG_under_control);
  } else {
    // 如果参数服务器中不存在名为/under_control的参数，则默认进入非控模式
    FLAG_under_control = false;
  }

  // 初始化基础订阅器和发布器
  init();
  
  // 逻辑判断部分
  if (FLAG_under_control) {
    // 如果节点处在控制节点的控制下，则等待控制节点发布跟踪目标(yolo/openpose/color/face)，跟踪对象(target name)
    ROS_INFO("[TargetTracker] Waiting for command from control node...");
  } else {
    // 如果节点不处在控制节点控制下，首先确认是否存在控制台输入参数
    if (argc == 5) {
      // 控制台参数格式 -track <yolo/openpose/color/face> -target <object name/human id/color/face id>
      if (std::string(argv[1]) == "-track") {
        if (std::string(argv[3]) == "-target") {
          track_ = std::string(argv[2]);
          targetName_ = std::string(argv[4]);
        }
      }
      setTrackTarget();
      ROS_INFO("[TargetTracker] Received track target: %s %s", argv[2], argv[4]);
    } else if(nodeHandle_.hasParam("/target_tracker/track") && nodeHandle_.hasParam("/target_tracker/target")){
        //如果没有控制台的输入参数，检查rosparam服务器中是否存在参数
        nodeHandle_.getParam("/target_tracker/track", track_);
        nodeHandle_.getParam("/target_tracker/target", targetName_);
        ROS_INFO("[TargetTracker] Read track target: %s %s from rosparam", track_.c_str(), targetName_.c_str());
        setTrackTarget();
    } else {
      ROS_ERROR ("[TargetTracker] Lack of parameters! Check for you rosparam!");
    }
  }
}

TargetShift::~TargetShift() {
  ROS_INFO("[TargetTracker] Shutting down...");
}

void TargetShift::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
  if (centerX_ == 0 && centerY_ == 0) {
    centerX_ = int(msg->width/2);
    centerY_ = int(msg->height/2);
    frameId_ = msg->header.frame_id;
    ROS_INFO("[CameraInfo] Setting pixel_center_x: %d, pixel_center_y:%d", centerX_, centerY_);
  }
}

void TargetShift::controlCallback(const robot_control_msgs::MissionConstPtr &msg) {
  if (msg->action == "track") {
    if (msg->target == "yolo") {
      assert(msg->attributes.object.name != "");
      track_ = msg->target;
      targetName_ = msg->attributes.object.name; 
      setTrackTarget();
    }
    else if (msg->target == "openpose") {
      // track human id
      assert(msg->attributes.human.name != "");
      track_ = msg->target;
      targetName_ = msg->attributes.human.name; 
      setTrackTarget();
    }
  }
}

void TargetShift::doneCallback(const actionlib::SimpleClientGoalState &state, const robot_navigation_msgs::MoveRobotResultConstPtr &result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  FLAG_base_focused = true;
}


void TargetShift::yoloTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const robot_vision_msgs::BoundingBoxesConstPtr &msg) {
  int targetIndex_ = 0;
  for (int i = 0; i < msg->bounding_boxes.size(); i++) {
    if (msg->bounding_boxes[i].Class == targetName_) {
      // 如果检测到目标，记录目标的index，并将flag置为true
      targetIndex_ = i;
      FLAG_target_found = true;
      FLAG_start_track = true;
      dynamixelControl(pan_state->current_pos, lift_state->current_pos);
      break;
    }
  }
  // 如果没有检测到目标
  if (!FLAG_target_found) {
    if (searchTargetCount_ == 3) {
      // 在平视状态下头部已经转过一周，设置头部向下观察
      if (fabs(lift_state->goal_pos - 0.43) > 0.1) {
        // 只发布一次lift关节控制指令
        dynamixelControl(0.0, 0.43);
      }
      // 等待lift关节完成移动，然后将搜索次数置0
      if (fabs(lift_state->current_pos - lift_state->goal_pos) <= 0.1) {
        searchTargetCount_ = 0;
      }
    }
    // 没有检测到目标时，转动头部搜索目标
    if ((fabs(pan_state->current_pos - pan_state->goal_pos) <= 0.1) && (searchTargetCount_ < 3)) {
      ROS_INFO("[TargetTracker] Searching for target: %s ...", targetName_.c_str());
      searchTargetCount_ += 1;
      // 此时舵机并不在运动中
      /* searchFrameCount： 1 -> pan_angle: -2.610
       * searchFrameCount: 2 -> pan_angle: 2.610
       */
      float pan_angle = pow((-1), searchTargetCount_)*(searchTargetCount_%3)*(2.610);
      dynamixelControl(pan_angle, lift_state->current_pos);
    }
  }

  if (FLAG_start_track) {
    std::cout << "======================= Enter Sync callback ===========================" << std::endl;
    if (!msg->bounding_boxes.empty()) {
      if (buffer.empty()) {
        buffer.push_back(int((msg->bounding_boxes[targetIndex_].ymax + msg->bounding_boxes[targetIndex_].ymin) / 2));
        buffer.push_back(int((msg->bounding_boxes[targetIndex_].ymax + msg->bounding_boxes[targetIndex_].ymin) / 2));
        buffer.push_back(int((msg->bounding_boxes[targetIndex_].ymax + msg->bounding_boxes[targetIndex_].ymin) / 2));
      }
      // 让index在0-3间循环
      filterIndex_ = (filterIndex_ + 1) % 3;
      // receive motor state
      currPanJointState_ = pan_state->current_pos;
      currLiftJointState_ = lift_state->current_pos;
      // searching for target object
      // 保留上一帧中的识别框中心点
      preCurrX_ = currX_;
      preCurrY_ = int((buffer[0] + buffer[1] + buffer[2])/3); // 对识别中心的Y方向坐标进行滤波，减少抖动带来的影响
      // 记录当前识别框的中心点
      currX_ = int((msg->bounding_boxes[targetIndex_].xmax + msg->bounding_boxes[targetIndex_].xmin) / 2);
      currY_ = int((msg->bounding_boxes[targetIndex_].ymax + msg->bounding_boxes[targetIndex_].ymin) / 2);
      buffer[filterIndex_] = currY_;
      // 输出前一帧识别框和当前帧识别框
      printf("Pre frame: (%d, %d), current frame: (%d, %d)\n", preCurrX_, preCurrY_, currX_, currY_);
      // 计算两帧识别框中心间的距离
      float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
      // 如果两帧识别框中心间距离超过10像素，则认为这一帧间运动无法被忽略，控制头部相机跟随物体
      if (frame2frameDistance > 10) {
        // 如果帧间运动无法忽略，则将静止帧计数清零，并控制电机
        staticFrameCount_ = 0;
        dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
      }
      // 如果两帧识别框中心间距离不超过10像素
      else if (staticFrameCount_ == 6){
        // 如果累计静止帧等于6帧，则开始调整机器人底盘
        // 同时将静止帧计数增加1，避免重复调用移动机器人的服务
        staticFrameCount_ += 1;
        // 调整FLAG，开始对准底盘
        FLAG_turn_base = true;
      } else {
        // 如果累计静止帧不等于6帧，则将其+1
        staticFrameCount_ += 1;
      }
      // 在底盘对准物体后，等待相机平稳
      if (staticFrameCount_ == 3 && FLAG_base_focused) {
        std::cout << targetPointPublisher_.getTopic() << std::endl;
        robot_vision_msgs::PixelPoint msg;
        msg.header.stamp = ros::Time::now();
        msg.image_header.frame_id = frameId_;
        msg.pixel_x = currX_;
        msg.pixel_y = currY_;
        // 根据发布器是否订阅话题来判断是否需要发布目标位置
        if (!targetPointPublisher_.getTopic().empty()) {
          targetPointPublisher_.publish(msg);
        }
      }
      std::cout << staticFrameCount_ << std::endl;
      if (staticFrameCount_ == 7 && FLAG_turn_base && !FLAG_base_focused) {
        if (fabs(turnedAngle_) > 0.1) {
          ROS_INFO("[TargetTrack] Start focusing move base for %f radius...", turnedAngle_);
          robot_navigation_msgs::MoveRobotGoal goal;
          goal.angle = turnedAngle_;
          actionClient_.sendGoal(goal, 
                                boost::bind(&TargetShift::doneCallback, this, _1, _2));
          FLAG_turn_base = false;
        } else {
          staticFrameCount_ = 0;
          FLAG_turn_base = false;
          FLAG_base_focused = true;
        }
      }
    }
  }
}

void TargetShift::colorTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const opencv_apps::RotatedRectStampedConstPtr &msg) {
  if (FLAG_start_track) {
    std::cout << "======================= Enter Sync callback ===========================" << std::endl;
    // receive motor state
    currPanJointState_ = pan_state->current_pos;
    currLiftJointState_ = lift_state->current_pos;
    preCurrX_ = currX_;
    preCurrY_ = currY_;
    // 记录当前识别框的中心点
    currX_ = int(msg->rect.center.x);
    currY_ = int(msg->rect.center.y);
    printf("Pre frame: (%d, %d), current frame: (%d, %d)\n", preCurrX_, preCurrY_, currX_, currY_);
    float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
    if (frame2frameDistance > 10) {
      staticFrameCount_ = 0;
      dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
    }
    else if (staticFrameCount_ == 6){
      staticFrameCount_ += 1;
      FLAG_turn_base = true;
    } else {
      staticFrameCount_ += 1;
    }
    if (staticFrameCount_ == 3 && FLAG_base_focused) {
      robot_vision_msgs::PixelPoint msg;
      msg.header.stamp = ros::Time::now();
      msg.image_header.frame_id = frameId_;
      msg.pixel_x = currX_;
      msg.pixel_y = currY_;
      if (!targetPointPublisher_.getTopic().empty()) {
        targetPointPublisher_.publish(msg);
      }
    }

    if (staticFrameCount_ == 7 && FLAG_turn_base && !FLAG_base_focused) {
      if (fabs(turnedAngle_) > 0.1) {
        ROS_INFO("[TargetTrack] Start focusing move base for %f radius...", turnedAngle_);
        robot_navigation_msgs::MoveRobotGoal goal;
        goal.angle = turnedAngle_;
        actionClient_.sendGoal(goal, 
                               boost::bind(&TargetShift::doneCallback, this, _1, _2));
        FLAG_turn_base = false;
      } else {
        staticFrameCount_ = 0;
        FLAG_turn_base = false;
        FLAG_base_focused = true;
      }
    }
  }
}

void TargetShift::faceTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const opencv_apps::FaceArrayStampedConstPtr &msg) {
  if (FLAG_start_track) {
    std::cout << "======================= Enter Sync callback ===========================" << std::endl;
    // receive motor state
    currPanJointState_ = pan_state->current_pos;
    currLiftJointState_ = lift_state->current_pos;
    // 记录当前识别框的中心点
    if (msg->faces.size() > 0) {
      preCurrX_ = currX_;
      preCurrY_ = currY_;
      int face_id = atoi(targetName_.c_str());
      currX_ = int(msg->faces[face_id].face.x);
      currY_ = int(msg->faces[face_id].face.y);
      printf("Pre frame: (%d, %d), current frame: (%d, %d)\n", preCurrX_, preCurrY_, currX_, currY_);
      float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
      if (frame2frameDistance > 10) {
        staticFrameCount_ = 0;
        dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
      }
      else if (staticFrameCount_ == 6){
        staticFrameCount_ += 1;
        FLAG_turn_base = true;
      } else {
        staticFrameCount_ += 1;
      }
    }
    if (staticFrameCount_ == 3 && FLAG_base_focused) {
      robot_vision_msgs::PixelPoint msg;
      msg.header.stamp = ros::Time::now();
      msg.image_header.frame_id = frameId_;
      msg.pixel_x = currX_;
      msg.pixel_y = currY_;
      if (!targetPointPublisher_.getTopic().empty()) {
        targetPointPublisher_.publish(msg);
      }
    }

    if (staticFrameCount_ == 7 && FLAG_turn_base && !FLAG_base_focused) {
      if (fabs(turnedAngle_) > 0.1) {
        ROS_INFO("[TargetTrack] Start focusing move base for %f radius...", turnedAngle_);
        robot_navigation_msgs::MoveRobotGoal goal;
        goal.angle = turnedAngle_;
        actionClient_.sendGoal(goal, 
                               boost::bind(&TargetShift::doneCallback, this, _1, _2));
        FLAG_turn_base = false;
      } else {
        staticFrameCount_ = 0;
        FLAG_turn_base = false;
        FLAG_base_focused = true;
      }
    }
  }
}

void TargetShift::faceWithNameTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const opencv_apps::FaceArrayStampedConstPtr &msg) {
  int targetIndex_ = 0;
  for (int i = 0; i < msg->faces.size(); i++) {
    if (msg->faces[i].label == targetName_) {
      // 如果检测到目标，记录目标的index，并将flag置为true
      targetIndex_ = i;
      FLAG_target_found = true;
      FLAG_start_track = true;
      dynamixelControl(pan_state->current_pos, 0.0);
      break;
    }
  }
  // 如果没有检测到目标
  if (!FLAG_target_found) {
    // 没有检测到目标时，转动头部搜索目标
    if (fabs(pan_state->current_pos - pan_state->goal_pos) <= 0.1) {
      std::cout << (pan_state->current_pos - pan_state->goal_pos) << std::endl;
      ROS_INFO("[TargetTracker] Searching for target: %s ...", targetName_.c_str());
      searchTargetCount_ += 1;
      // 此时舵机并不在运动中
      float pan_angle = pow((-1), searchTargetCount_)*(searchTargetCount_%3)*(2.610);
      dynamixelControl(pan_angle, 0.0);
    }
  }

  if (FLAG_start_track) {
    if (!msg->faces.empty()) {
      std::cout << "======================= Enter Sync callback ===========================" << std::endl;
      if (buffer.empty()) {
        buffer.push_back(int(msg->faces[targetIndex_].face.y));
        buffer.push_back(int(msg->faces[targetIndex_].face.y));
        buffer.push_back(int(msg->faces[targetIndex_].face.y));
      }
      filterIndex_ = (filterIndex_+1)%3;
      // receive motor state
      currPanJointState_ = pan_state->current_pos;
      currLiftJointState_ = lift_state->current_pos;
      // 记录当前识别框的中心点
      preCurrX_ = currX_;
      preCurrY_ = int((buffer[0] + buffer[1] + buffer[2])/3);
      currX_ = int(msg->faces[targetIndex_].face.x);
      currY_ = int(msg->faces[targetIndex_].face.y);
      buffer[filterIndex_] = currY_;
      printf("Pre frame: (%d, %d), current frame: (%d, %d)\n", preCurrX_, preCurrY_, currX_, currY_);
      float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
      if (frame2frameDistance > 10) {
        staticFrameCount_ = 0;
        dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
      }
      else if (staticFrameCount_ == 6){
        staticFrameCount_ += 1;
        FLAG_turn_base = true;
      } else {
        staticFrameCount_ += 1;
      }
      if (staticFrameCount_ == 3 && FLAG_base_focused) {
        robot_vision_msgs::PixelPoint msg;
        msg.header.stamp = ros::Time::now();
        msg.image_header.frame_id = frameId_;
        msg.pixel_x = currX_;
        msg.pixel_y = currY_;
        if (!targetPointPublisher_.getTopic().empty()) {
          targetPointPublisher_.publish(msg);
        }
      }

      if (staticFrameCount_ == 7 && FLAG_turn_base && !FLAG_base_focused) {
        if (fabs(turnedAngle_) > 0.1) {
          ROS_INFO("[TargetTrack] Start focusing move base for %f radius...", turnedAngle_);
          robot_navigation_msgs::MoveRobotGoal goal;
          goal.angle = turnedAngle_;
          actionClient_.sendGoal(goal, 
                                boost::bind(&TargetShift::doneCallback, this, _1, _2));
          FLAG_turn_base = false;
        } else {
          staticFrameCount_ = 0;
          FLAG_turn_base = false;
          FLAG_base_focused = true;
        }
      }
    }
  }
}


void TargetShift::openposeTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const robot_vision_msgs::HumanPosesConstPtr &msg) {
  int targetIndex_ = 0;
  for (int i = 0; i < msg->poses.size(); i++) {
    if (msg->poses[i].pose == targetName_) {
      // 如果检测到目标，记录目标的index，并将flag置为true
      targetIndex_ = i;
      FLAG_target_found = true;
      FLAG_start_track = true;
      dynamixelControl(pan_state->current_pos, 0.0);
      break;
    }
  }
  // 如果没有检测到目标
  if (!FLAG_target_found) {
    // 没有检测到目标时，转动头部搜索目标
    if (fabs(pan_state->current_pos - pan_state->goal_pos) <= 0.1) {
      std::cout << (pan_state->current_pos - pan_state->goal_pos) << std::endl;
      ROS_INFO("[TargetTracker] Searching for target: %s ...", targetName_.c_str());
      searchTargetCount_ += 1;
      // 此时舵机并不在运动中
      float pan_angle = pow((-1), searchTargetCount_)*(searchTargetCount_%3)*(2.610);
      dynamixelControl(pan_angle, 0.0);
    }
  }
  if (FLAG_start_track) {
    if (!msg->poses.empty()) {
      std::cout << "======================= Enter Sync callback ===========================" << std::endl;
      if (buffer.empty()) {
        buffer.push_back(int(msg->poses[targetIndex_].Chest.y));
        buffer.push_back(int(msg->poses[targetIndex_].Chest.y));
        buffer.push_back(int(msg->poses[targetIndex_].Chest.y));
      }
      filterIndex_ = (filterIndex_ + 1)%3;
      // receive motor state
      currPanJointState_ = pan_state->current_pos;
      currLiftJointState_ = lift_state->current_pos;
      // searching for target object
      // 保留上一帧中的识别框中心点
      preCurrX_ = currX_;
      preCurrY_ = (buffer[0] + buffer[1] + buffer[2])/3;
      // 记录当前识别框的中心点
      if (msg->poses[targetIndex_].Chest.x != -1 && msg->poses[targetIndex_].Chest.y != -1) {
        // 如果识别到了胸部的关节点，则更新当前中心点x,y坐标
        currX_ = int(msg->poses[targetIndex_].Chest.x);
        currY_ = int(msg->poses[targetIndex_].Chest.y);
      } else {
        // 如果没有识别到，则保持上一次识别结果不变，防止摄像头跳动
        currX_ = preCurrX_;
        currY_ = preCurrY_;
      }
      
      buffer[filterIndex_] = currY_;
      printf("Pre frame: (%d, %d), current frame: (%d, %d)\n", preCurrX_, preCurrY_, currX_, currY_);
      float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
      if (frame2frameDistance > 10) {
        staticFrameCount_ = 0;
        dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
      }
      else if (staticFrameCount_ == 6){
        staticFrameCount_ += 1;
        FLAG_turn_base = true;
      } else {
        staticFrameCount_ += 1;
      }

      if (staticFrameCount_ == 3 && FLAG_base_focused) {
        robot_vision_msgs::PixelPoint msg;
        msg.header.stamp = ros::Time::now();
        msg.image_header.frame_id = frameId_;
        msg.pixel_x = currX_;
        msg.pixel_y = currY_;
        if (!targetPointPublisher_.getTopic().empty()) {
          targetPointPublisher_.publish(msg);
        }
      }

      if (staticFrameCount_ == 7 && FLAG_turn_base && !FLAG_base_focused) {
        if (fabs(turnedAngle_) > 0.1) {
          ROS_INFO("[TargetTrack] Start focusing move base for %f radius...", turnedAngle_);
          robot_navigation_msgs::MoveRobotGoal goal;
          goal.angle = turnedAngle_;
          actionClient_.sendGoal(goal, 
                                boost::bind(&TargetShift::doneCallback, this, _1, _2));
          FLAG_turn_base = false;
        } else {
          staticFrameCount_ = 0;
          FLAG_turn_base = false;
          FLAG_base_focused = true;
        }
      }
    }
  }
}


void TargetShift::dynamixelControl(int curr_x, int curr_y, float pan_state, float lift_state, float scale) {
  int error_x = centerX_ - curr_x;
  int error_y = centerY_ - curr_y;
  printf("X error: %d, Y error: %d\n", error_x, error_y);
  turnedAngle_ = pan_state + error_x*scale;

  float next_state_pan =  pan_state + error_x*scale;
  float next_state_lift = lift_state - error_y*scale;
  printf("Current pan: %f, lift: %f -> Next pan: %f, lift: %f\n",pan_state, lift_state, next_state_pan, next_state_lift);
  
  std_msgs::Float64 msg_pan;
  std_msgs::Float64 msg_lift;

  msg_pan.data = next_state_pan;
  msg_lift.data = next_state_lift;

  headPanJointPublisher_.publish(msg_pan);
  headLiftJointPublisher_.publish(msg_lift);
}

void TargetShift::dynamixelControl(float pan_angle, float lift_angle) {
  std_msgs::Float64 msg_pan;
  std_msgs::Float64 msg_lift;

  msg_pan.data = pan_angle;
  msg_lift.data = lift_angle;
  
  headPanJointPublisher_.publish(msg_pan);
  headLiftJointPublisher_.publish(msg_lift);
}

void TargetShift::init() {
  std::string cameraInfoTopicName_;
  int cameraInfoQueueSize_;

  nodeHandle_.param("subscribers/camera_info/topic", cameraInfoTopicName_, std::string("/camera/rgb/camera_info"));
  nodeHandle_.param("subscribers/camera_info/queue_size", cameraInfoQueueSize_, 1);

  // 初始化接收相机属性的订阅器
  cameraInfoSubscriber_   = nodeHandle_.subscribe(cameraInfoTopicName_, 1, &TargetShift::cameraInfoCallback, this);
  // 初始化发布电机关节角度的publisher
  headPanJointPublisher_  = nodeHandle_.advertise<std_msgs::Float64>("/head_pan_joint/command", 1, false);
  headLiftJointPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("/head_lift_joint/command", 1, false);

  // 如果处于控制节点控制，则初始化一系列与控制节点相关的订阅器和发布器
  if (FLAG_under_control) {
    std::string cameraInfoTopicName_;
    int cameraInfoQueueSize_;

    std::string controlSubTopicName_;
    int controlSubQueueSize_;

    std::string controlPubTopicName_;
    int controlPubQueueSize_;
    bool controlPubLatch_;

    nodeHandle_.param("subscribers/camera_info/topic", cameraInfoTopicName_, std::string("/camera/rgb/camera_info"));
    nodeHandle_.param("subscribers/camera_info/queue_size", cameraInfoQueueSize_, 1);
    nodeHandle_.param("subscribers/control_to_vision/topic", controlSubTopicName_, std::string("/control_to_vision"));
    nodeHandle_.param("subscribers/control_to_vision/queue_size", controlSubQueueSize_, 1);
    nodeHandle_.param("publishers/vision_to_control/topic", controlPubTopicName_, std::string("/vision_to_control"));
    nodeHandle_.param("publishers/vision_to_control/queue_size", controlSubQueueSize_, 1);
    nodeHandle_.param("publishers/vision_to_control/latch", controlPubLatch_, bool(false));

    // 初始化订阅control节点命令的订阅器
    controlSubscriber_  = nodeHandle_.subscribe(controlSubTopicName_, 1, &TargetShift::controlCallback, this);
    // 初始化向control节点发送反馈的发布器
    controlPublisher_   = nodeHandle_.advertise<robot_control_msgs::Feedback>(controlPubTopicName_, controlPubQueueSize_, controlPubLatch_);
  } else {
    std::string targetPointPubTopicName_;
    int targetPointPubQueueSize_;
    bool targetPointPubLatch_;
    
    nodeHandle_.param("publishers/target_point/topic", targetPointPubTopicName_, std::string("/robot_vision/pixel_point"));
    nodeHandle_.param("publishers/target_point/queue_size", targetPointPubQueueSize_, 1);
    nodeHandle_.param("publishers/target_point/latch", targetPointPubLatch_, bool(false));

    targetPointPublisher_ = nodeHandle_.advertise<robot_vision_msgs::PixelPoint>(targetPointPubTopicName_, targetPointPubQueueSize_, targetPointPubLatch_);

  }
}

void TargetShift::setTrackTarget() {
  // Initialize parameters
  staticFrameCount_ = 0;
  turnedAngle_ = 0;
  if (track_ == "yolo") {
    ROS_INFO("[TargetTracker] Setting Synchronizer <JointState, JointState, bounding_boxes>, Callback: yoloTrackCallback");

    // create subscribers with message filter
    panJointSubscriber_.subscribe(nodeHandle_, "/head_pan_joint/state", 1, ros::TransportHints().tcpNoDelay());
    liftJointSubscriber_.subscribe(nodeHandle_, "/head_lift_joint/state", 1, ros::TransportHints().tcpNoDelay());
    bboxSubscriber_.subscribe(nodeHandle_, "/yolo_ros/bounding_boxes", 1, ros::TransportHints().tcpNoDelay());

    // Declare synchronizer and set time approximate to be 10
    yoloSync.reset(new YoloSync(YoloSyncPolicy(10), panJointSubscriber_, liftJointSubscriber_, bboxSubscriber_));
    yoloSync->registerCallback(boost::bind(&TargetShift::yoloTrackCallback, this, _1, _2, _3));
  } 

  else if (track_ == "openpose") {
    ROS_INFO("[TargetTracker] Setting Synchronizer <JointState, JointState, human_poses>, Callback: openposeTrackCallback");

    // create subscribers with message filter
    panJointSubscriber_.subscribe(nodeHandle_, "/head_pan_joint/state", 1, ros::TransportHints().tcpNoDelay());
    liftJointSubscriber_.subscribe(nodeHandle_, "/head_lift_joint/state", 1, ros::TransportHints().tcpNoDelay());
    poseSubscriber_.subscribe(nodeHandle_, "/openpose_ros/human_poses", 1, ros::TransportHints().tcpNoDelay());

    // Declare synchronizer and set time approximate to be 10
    openposeSync.reset(new OpenposeSync(OpenposeSyncPolicy(10), panJointSubscriber_, liftJointSubscriber_, poseSubscriber_));
    openposeSync->registerCallback(boost::bind(&TargetShift::openposeTrackCallback, this, _1, _2, _3));
  }

  else if (track_ == "color") {
    ROS_INFO("[TargetTracker] Setting Synchronizer <JointState, JointState, color_box>, Callback: colorTrackCallback");

    // create subscribers with message filter
    panJointSubscriber_.subscribe(nodeHandle_, "/head_pan_joint/state", 1, ros::TransportHints().tcpNoDelay());
    liftJointSubscriber_.subscribe(nodeHandle_, "/head_lift_joint/state", 1, ros::TransportHints().tcpNoDelay());
    colorSubscriber_.subscribe(nodeHandle_, "/camshift/track_box", 1, ros::TransportHints().tcpNoDelay());

    // Declare synchronizer and set time approximate to be 10
    colorSync.reset(new ColorSync(ColorSyncPolicy(10), panJointSubscriber_, liftJointSubscriber_, colorSubscriber_));
    colorSync->registerCallback(boost::bind(&TargetShift::colorTrackCallback, this, _1, _2, _3));
  }

  else if (track_ == "face") {
    ROS_INFO("[TargetTracker] Setting Synchronizer <JointState, JointState, face_box>, Callback: faceTrackCallback");

    // create subscribers with message filter
    panJointSubscriber_.subscribe(nodeHandle_, "/head_pan_joint/state", 1, ros::TransportHints().tcpNoDelay());
    liftJointSubscriber_.subscribe(nodeHandle_, "/head_lift_joint/state", 1, ros::TransportHints().tcpNoDelay());
    faceSubscriber_.subscribe(nodeHandle_, "/face_detection/faces", 1, ros::TransportHints().tcpNoDelay());

    // Declare synchronizer and set time approximate to be 10
    faceSync.reset(new FaceSync(FaceSyncPolicy(10), panJointSubscriber_, liftJointSubscriber_, faceSubscriber_));
    faceSync->registerCallback(boost::bind(&TargetShift::faceTrackCallback, this, _1, _2, _3));
  }

  else if (track_ == "person") {
    ROS_INFO("[TargetTracker] Setting Synchronizer <JointState, JointState, face_box>, Callback: faceWithNameTrackCallback");

    // create subscribers with message filter
    panJointSubscriber_.subscribe(nodeHandle_, "/head_pan_joint/state", 1, ros::TransportHints().tcpNoDelay());
    liftJointSubscriber_.subscribe(nodeHandle_, "/head_lift_joint/state", 1, ros::TransportHints().tcpNoDelay());
    faceSubscriber_.subscribe(nodeHandle_, "/face_recognition/output", 1, ros::TransportHints().tcpNoDelay());

    // Declare synchronizer and set time approximate to be 10
    faceSync.reset(new FaceSync(FaceSyncPolicy(10), panJointSubscriber_, liftJointSubscriber_, faceSubscriber_));
    faceSync->registerCallback(boost::bind(&TargetShift::faceWithNameTrackCallback, this, _1, _2, _3));
  }
}

int main(int argc ,char** argv) {
  ros::init(argc, argv, "target_tracker");
  ros::NodeHandle nh("~");
  target_tracker::TargetShift tracker(nh, argc, argv);

  ros::spin();
  return 0;
}