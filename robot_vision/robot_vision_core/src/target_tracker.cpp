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

TargetShift::TargetShift(ros::NodeHandle nh, int argc, char** argv): nodeHandle_(nh) {
  ROS_INFO("[TargetTracker] Initializing...");
  // Initialize basic subscribers and publishers
  init();
  if (argc == 5) {
    // -track <yolo/openpose/color/face>
    if (std::string(argv[1]) == "-track") {
      track_ = std::string(argv[2]);
      // -target <object name/human id/color/face id>
      if (std::string(argv[3]) == "-target") {
        targetName_ = std::string(argv[4]);
      }
    }
    ROS_INFO("[TargetTracker] Received track target: %s %s", argv[2], argv[4]);
    setTrackTarget();
  } else {
    ROS_INFO("[TargetTracker] Waiting for command from control node...");
  }
}

TargetShift::~TargetShift() {
  ROS_INFO("[TargetTracker] Shutting down...");
}

void TargetShift::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg) {
  if (centerX_ == 0 && centerY_ == 0) {
    centerX_ = int(msg->width/2);
    centerY_ = int(msg->height/2);
    FLAG_start_track = true;
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

void TargetShift::yoloTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const robot_vision_msgs::BoundingBoxesConstPtr &msg) {
  if (FLAG_start_track) {
    std::cout << "======================= Enter Sync callback ===========================" << std::endl;
    // receive motor state
    currPanJointState_ = pan_state->current_pos;
    currLiftJointState_ = lift_state->current_pos;
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
        float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
        if (frame2frameDistance > 10) {
          dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
        }
        // 如果识别到多个目标物体怎么办。。。
        // 目前尝试方案：识别到目标物体后直接break出当前循环
        break;
      }
    }
    
    // float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
    // if (frame2frameDistance > 10) {
    //   dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
    // }
  }
}

void TargetShift::colorTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const opencv_apps::RotatedRectStampedConstPtr &msg) {
  if (FLAG_start_track) {
    std::cout << "======================= Enter Sync callback ===========================" << std::endl;
    // receive motor state
    currPanJointState_ = pan_state->current_pos;
    currLiftJointState_ = lift_state->current_pos;
    preCurrX_ = currX_;
    preCurrY_ = currX_;
    // 记录当前识别框的中心点
    currX_ = int(msg->rect.center.x);
    currY_ = int(msg->rect.center.y);
    printf("Pre frame: (%d, %d), current frame: (%d, %d)\n", preCurrX_, preCurrY_, currX_, currY_);
    float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
    if (frame2frameDistance > 10) {
      dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
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
      preCurrY_ = currX_;
      int face_id = atoi(targetName_.c_str());
      currX_ = int(msg->faces[face_id].face.x);
      currY_ = int(msg->faces[face_id].face.y);
      printf("Pre frame: (%d, %d), current frame: (%d, %d)\n", preCurrX_, preCurrY_, currX_, currY_);
      float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
      if (frame2frameDistance > 10) {
        dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
      }
    }
  }
}

void TargetShift::faceWithNameTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const opencv_apps::FaceArrayStampedConstPtr &msg) {
  if (FLAG_start_track) {
    std::cout << "======================= Enter Sync callback ===========================" << std::endl;
    // receive motor state
    currPanJointState_ = pan_state->current_pos;
    currLiftJointState_ = lift_state->current_pos;
    // 记录当前识别框的中心点
    for (int i=0; i<msg->faces.size(); i++) {
      if (msg->faces[i].label == std::string(targetName_)) {
        preCurrX_ = currX_;
        preCurrY_ = currX_;
        currX_ = int(msg->faces[i].face.x);
        currY_ = int(msg->faces[i].face.y);
        printf("Pre frame: (%d, %d), current frame: (%d, %d)\n", preCurrX_, preCurrY_, currX_, currY_);
        float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
        if (frame2frameDistance > 10) {
          dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
        }
      }
    }
  }
}


void TargetShift::openposeTrackCallback(const dynamixel_msgs::JointStateConstPtr &pan_state, const dynamixel_msgs::JointStateConstPtr &lift_state, const robot_vision_msgs::HumanPosesConstPtr &msg) {
  if (FLAG_start_track) {
    std::cout << "======================= Enter Sync callback ===========================" << std::endl;
    // receive motor state
    currPanJointState_ = pan_state->current_pos;
    currLiftJointState_ = lift_state->current_pos;
    // searching for target object
    for (int i = 0; i < msg->poses.size(); i++) {
      if (std::to_string(msg->poses[i].human_id) == targetName_) {
        // 保留上一帧中的识别框中心点
        preCurrX_ = currX_;
        preCurrY_ = currX_;
        // 记录当前识别框的中心点
        currX_ = int(msg->poses[i].Chest.x);
        currY_ = int(msg->poses[i].Chest.y);
        printf("Pre frame: (%d, %d), current frame: (%d, %d)\n", preCurrX_, preCurrY_, currX_, currY_);
        float frame2frameDistance = calcPixelDistance(preCurrX_, preCurrY_, currX_, currY_);
        if (frame2frameDistance > 10) {
          dynamixelControl(currX_, currY_, currPanJointState_, currLiftJointState_, 0.001);
        }
        break;
      }
    }
  }
}

void TargetShift::dynamixelControl(int curr_x, int curr_y, float pan_state, float lift_state, float scale) {
  int error_x = centerX_ - curr_x;
  int error_y = centerY_ - curr_y;
  printf("X error: %d, Y error: %d\n", error_x, error_y);

  float next_state_pan =  pan_state + error_x*scale;
  float next_state_lift = lift_state - error_y*scale;
  printf("Current pan: %f, lift: %f -> Next pan: %f, lift: %f",pan_state, lift_state, next_state_pan, next_state_lift);
  
  std_msgs::Float64 msg_pan;
  std_msgs::Float64 msg_lift;

  msg_pan.data = next_state_pan;
  msg_lift.data = next_state_lift;

  headPanJointPublisher_.publish(msg_pan);
  headLiftJointPublisher_.publish(msg_lift);
}

void TargetShift::init() {
  std::string cameraInfoTopicName_;
  int cameraInfoQueueSize_;

  std::string controlTopicName_;
  int controlQueueSize_;

  nodeHandle_.param("subscribers/camera_info/topic", cameraInfoTopicName_, std::string("/camera/rgb/camera_info"));
  nodeHandle_.param("subscribers/camera_info/queue_size", cameraInfoQueueSize_, 1);
  nodeHandle_.param("subscribers/control_to_vision/topic", controlTopicName_, std::string("/control_to_vision"));
  nodeHandle_.param("subscribers/control_to_vision/queue_size", controlQueueSize_, 1);

  headPanJointPublisher_  = nodeHandle_.advertise<std_msgs::Float64>("/head_pan_joint/command", 1, false);
  headLiftJointPublisher_ = nodeHandle_.advertise<std_msgs::Float64>("/head_lift_joint/command", 1, false);
  cameraInfoSubscriber_   = nodeHandle_.subscribe(cameraInfoTopicName_, 1, &TargetShift::cameraInfoCallback, this);
  controlSubscriber_      = nodeHandle_.subscribe(controlTopicName_, 1, &TargetShift::controlCallback, this);
}

void TargetShift::setTrackTarget() {
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