/*
 * target_localization.cpp
 * 
 *  Created on: Aug 26th, 2020
 *      Author: Hilbert Xu
 *   Institute: Mustar Robot
 *   使用前修改头部电机速度为0.1
 */

#include <robot_vision_core/target_localization.hpp>

using namespace target_localization;

TargetLocate::TargetLocate(ros::NodeHandle nh, int argc, char** argv):nodeHandle_(nh), actionClient_("/move_robot_server", true) {
  // 等待控制机器人移动的服务端启动
  ROS_INFO("[TargetLocate] Waiting for MoveRobotServer...");
  actionClient_.waitForServer();
  
  // 从rosparam参数服务器中读取参数
  if (nodeHandle_.hasParam("/under_control")) {
    // 如果存在/under_control参数
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
    ROS_INFO("[TargetLocate] Waiting for command from control node...");
  } else {
    // 如果节点不处在控制节点的控制下，首先确认是否存在控制台输入参数
    if (argc == 5) {
      // 控制台参数格式 -source_frame <frame_name> -target_frame <frame_name>
      if (std::string(argv[1]) == "-source_frame") {
        if (std::string(argv[3]) == "-target_frame") {
          sourceFrame_ = std::string(argv[2]);
          targetFrame_ = std::string(argv[4]);
          ROS_INFO("[TargetLocate] Transform from: %s -> %s", argv[2], argv[4]);
        }
      }
    } else if (nodeHandle_.hasParam("/target_localization/source_frame") && nodeHandle_.hasParam("/target_localization/target_frame")) {
      //如果没有控制台的输入参数，检查rosparam服务器中是否存在参数
      nodeHandle_.getParam("/target_localization/source_frame", sourceFrame_);
      nodeHandle_.getParam("/target_localization/target_frame", targetFrame_);
      ROS_INFO("[TargetLocate] Transform from: %s -> %s", sourceFrame_.c_str(), targetFrame_.c_str());
    } else {
      ROS_ERROR ("[TargetLocate] Lack of parameters! Check for you rosparam!");
    }
  }
}

TargetLocate::~TargetLocate() {
  ROS_INFO("[TargetLocate] Shutting down...");
}

void TargetLocate::init () {
  std::string cameraInfoTopicName_;
  int cameraInfoQueueSize_;

  std::string pclTopicName_;
  int pclQueueSize_;

  std::string imageTopicName_;
  int imageQueueSize_;

  nodeHandle_.param("subscribers/camera_info/topic", cameraInfoTopicName_, std::string("/camera_top/rgb/camera_info"));
  nodeHandle_.param("subscribers/camera_info/queue_size", cameraInfoQueueSize_, 1);
  nodeHandle_.param("subscribers/camera_depth/topic", pclTopicName_, std::string("/camera_top/depth/points"));
  nodeHandle_.param("subscribers/camera_depth/queue_size", pclQueueSize_, 1);
  nodeHandle_.param("subscribers/camera_rgb/topic", imageTopicName_, std::string("/camera_top/rgb/image_raw"));
  nodeHandle_.param("subscribers/camera_rgb/queue_size", imageQueueSize_, 1);

  pclSubscriber_        = nodeHandle_.subscribe(pclTopicName_,        pclQueueSize_,        &TargetLocate::pclCallback, this);
  imageSubscriber_      = nodeHandle_.subscribe(imageTopicName_,      imageQueueSize_,      &TargetLocate::imageCallback, this);
  cameraInfoSubscriber_ = nodeHandle_.subscribe(cameraInfoTopicName_, cameraInfoQueueSize_, &TargetLocate::cameraInfoCallback, this);

  if (FLAG_under_control) {
    std::string controlSubTopicName_;
    int controlSubQueueSize_;

    std::string controlPubTopicName_;
    int controlPubQueueSize_;
    bool controlPubLatch_;

    nodeHandle_.param("subscribers/control_to_vision/topic", controlSubTopicName_, std::string("/control_to_vision"));
    nodeHandle_.param("subscribers/control_to_vision/queue_size", controlSubQueueSize_, 1);
    nodeHandle_.param("publishers/vision_to_control/topic", controlPubTopicName_, std::string("/vision_to_control"));
    nodeHandle_.param("publishers/vision_to_control/queue_size", controlSubQueueSize_, 1);
    nodeHandle_.param("publishers/vision_to_control/latch", controlPubLatch_, bool(false));

    // 初始化订阅control节点命令的订阅器
    controlSubscriber_      = nodeHandle_.subscribe(controlSubTopicName_, 1, &TargetLocate::controlCallback, this);
    // 初始化向control节点发送反馈的发布器
    controlPublisher_       = nodeHandle_.advertise<robot_control_msgs::Feedback>(controlPubTopicName_, controlPubQueueSize_, controlPubLatch_);
  } else {
    std::string pixelPointTopicName_;
    int pixelPointQueueSize_;

    std::string spacePointTopicName_;
    int spacePointQueueSize_;
    bool spacePointLatch_;

    std::string naviPointTopic_;
    int naviPointQueueSize_;
    bool naviPointLatch_;

    nodeHandle_.param("subscribers/pixel_point/topic", pixelPointTopicName_, std::string("/robot_vision/pixel_point"));
    nodeHandle_.param("subscribers/pixel_point/queue_size", pixelPointQueueSize_, 1);
    nodeHandle_.param("publishers/space_point/topic", spacePointTopicName_, std::string("/robot_vision/space_point"));
    nodeHandle_.param("publishers/space_point/queue_size", spacePointQueueSize_, 1);
    nodeHandle_.param("publishers/space_point/latch", spacePointLatch_, false);
    nodeHandle_.param("publishers/navi_point/topic", naviPointTopic_, std::string("/robot_vision/navi_point"));
    nodeHandle_.param("publishers/navi_point/queue_size", naviPointQueueSize_, 1);
    nodeHandle_.param("publishers/navi_point/latch", naviPointLatch_, false);

    pixelPointSubscriber_ = nodeHandle_.subscribe(pixelPointTopicName_, pixelPointQueueSize_, &TargetLocate::pixelPointCallback, this);
    spacePointPublisher_  = nodeHandle_.advertise<robot_vision_msgs::SpacePoint>(spacePointTopicName_, spacePointQueueSize_, spacePointLatch_);
    naviPointPublisher_   = nodeHandle_.advertise<geometry_msgs::Pose>(naviPointTopic_, naviPointQueueSize_, naviPointLatch_);
  }

}

void TargetLocate::doneCallback(const actionlib::SimpleClientGoalState &state, const robot_navigation_msgs::MoveRobotResultConstPtr &result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  rotation = result->rotation;
}

int TargetLocate::findNearValid (int idx) {
  double temp_min = 9999999999;
  int return_idx = idx;
  int obj_row = idx/cameraWidth_;
  int obj_col = idx%cameraWidth_;

  for (int row=0; row<cameraHeight_; row++) {
    for (int col=0; col<cameraWidth_; col++) {
      if (!std::isnan(cloudAstra->points[row*cameraWidth_+col].x) &&
          !std::isnan(cloudAstra->points[row*cameraWidth_+col].y) &&
          !std::isnan(cloudAstra->points[row*cameraWidth_+col].z)) {
        double dis = (row-obj_row)*(row-obj_row) + (col-obj_col)*(col-obj_col);
        if (dis < temp_min) {
          return_idx = row*cameraWidth_ + col;
          temp_min = dis;
        }
      }
    }
  }
  validIndex_ = return_idx;
  return return_idx;
}

void TargetLocate::pixelPointCallback (const robot_vision_msgs::PixelPointConstPtr &msg) {
  if (object_x == 0 && object_y == 0) {
    object_x = msg->pixel_x;
    object_y = msg->pixel_y;
    
    robot_navigation_msgs::MoveRobotGoal goal;
    // 发送一个空的移动指令，读取当期机器人的姿态信息
    goal.angle = 0.0;
    goal.distance = 0.0;
    actionClient_.sendGoal(goal, 
                            boost::bind(&TargetLocate::doneCallback, this, _1, _2));
    FLAG_sub_pcl_data = true;
  }
}

void TargetLocate::cameraInfoCallback (const sensor_msgs::CameraInfoConstPtr &msg) {
  if (cameraWidth_ == 0 && cameraHeight_ == 0) {
    cameraHeight_ = msg->height;
    cameraWidth_  = msg->width;
  }
}

void TargetLocate::controlCallback (const robot_control_msgs::MissionConstPtr &msg) {
  if (msg->action == "locate") {
    object_x = msg->attributes.vision.pixel_coords.pixel_x;
    object_y = msg->attributes.vision.pixel_coords.pixel_y;
    std::cout << "Received pixel coordinates: " << "(" << object_x << "," << object_y << ")" << std::endl;
    if (msg->target == "object") {
      sourceFrame_ = "camera_top_depth_frame";
      targetFrame_ = "/base_link";
    } else if (msg->target == "human") {
      sourceFrame_ = "camera_top_depth_frame";
      targetFrame_ = "/map";
    }
    FLAG_sub_pcl_data = true;
  }
}

void TargetLocate::imageCallback (const sensor_msgs::ImageConstPtr &msg) {
  if (!cloudAstra->empty()) {
    int ori_row = originIndex_ / cameraWidth_;
    int ori_col = originIndex_ % cameraWidth_;
    int val_row = validIndex_ / cameraWidth_;
    int val_col = validIndex_ % cameraWidth_;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    cv::Point ori(ori_col, ori_row);
    cv::Point val(val_col, val_row);

    circle(img, ori, 3, cv::Scalar(0,0,255), -1);
    circle(img, val, 3, cv::Scalar(255,0,0), -1);

    cv::imshow("target_localization", img);
    cv::waitKey(10);
  }
}

void TargetLocate::pclCallback (sensor_msgs::PointCloud2 msg) {
  if (FLAG_sub_pcl_data == true) {
    // 储存原始点云数据
    pcl::fromROSMsg (msg, *cloudAstra);
  }
}

void TargetLocate::transformTarget() {
  if (cloudAstra->empty()) {
    ROS_INFO ("[TargetLocate] Waiting for pcl transform...");
    sleep (2);
  } else {
    geometry_msgs::PointStamped cam_pos;
    geometry_msgs::PointStamped base_point;
    cam_pos.header.frame_id = sourceFrame_;
    originIndex_ = object_y * cameraWidth_ + object_x;
    tf::TransformListener pListener;

    if (std::isnan(cloudAstra->points[originIndex_].x) || 
        std::isnan(cloudAstra->points[originIndex_].y) ||
        std::isnan(cloudAstra->points[originIndex_].z)) {  
      ROS_INFO("Fix invalid point cloud, Finding the nearest valid point");
      int new_index = findNearValid(originIndex_);
      cam_pos.header.stamp = ros::Time(0);
      cam_pos.point.x = cloudAstra->points[new_index].x;
      cam_pos.point.y = cloudAstra->points[new_index].y;
      cam_pos.point.z = cloudAstra->points[new_index].z;

      try {
        ROS_INFO("Listening for the tf transform");
        pListener.waitForTransform(sourceFrame_, targetFrame_, ros::Time(0), ros::Duration(3.0));
        pListener.transformPoint(targetFrame_, cam_pos, base_point);
        ROS_INFO("%s: (%.2f, %.2f. %.2f) -----> %s: (%.2f, %.2f, %.2f) at time %.2f",
                sourceFrame_.c_str(),
                cam_pos.point.x, cam_pos.point.y, cam_pos.point.z,
                targetFrame_.c_str(),
                base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
      }
      catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s",sourceFrame_.c_str(), targetFrame_.c_str(), ex.what());
      }
    } else {
      ROS_INFO("Congraz! Current point is valid");
      cam_pos.header.stamp = ros::Time(0);
      cam_pos.point.x = cloudAstra->points[originIndex_].x;
      cam_pos.point.y = cloudAstra->points[originIndex_].y;
      cam_pos.point.z = cloudAstra->points[originIndex_].z;
      try {
        ROS_INFO("Listening for the tf transform");
        pListener.waitForTransform(sourceFrame_, targetFrame_, ros::Time(0), ros::Duration(3.0));
        pListener.transformPoint(targetFrame_, cam_pos, base_point);
        ROS_INFO("%s: (%.2f, %.2f. %.2f) -----> %s: (%.2f, %.2f, %.2f) at time %.2f",
                sourceFrame_.c_str(),
                cam_pos.point.x, cam_pos.point.y, cam_pos.point.z,
                targetFrame_.c_str(),
                base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
      }
      catch(tf::TransformException& ex) {
        ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", sourceFrame_.c_str(), targetFrame_.c_str(), ex.what());
      }
    }

    if (FLAG_pub_obj_pos && FLAG_under_control) {
      // 控制发布器只发布一次消息
      robot_control_msgs::Feedback msg;
      msg.action = "locate";
      msg.target = "object";
      msg.mission_state = "success";
      msg.results.vision.header.frame_id = targetFrame_;
      msg.results.vision.space_coords.x = base_point.point.x;
      msg.results.vision.space_coords.y = base_point.point.y;
      msg.results.vision.space_coords.z = base_point.point.z;
      controlPublisher_.publish(msg);
      FLAG_pub_obj_pos = true;
    } else if (FLAG_pub_obj_pos && !FLAG_under_control) {

      robot_vision_msgs::SpacePoint msg;
      msg.header.stamp = ros::Time::now();
      msg.image_header.frame_id = targetFrame_;
      msg.space_x = 0.31;
      msg.space_y = base_point.point.y;
      msg.space_z = base_point.point.z;
      spacePointPublisher_.publish(msg);
      robot_navigation_msgs::MoveRobotGoal goal;
      goal.distance = base_point.point.x - 0.31;
      actionClient_.sendGoal(goal, 
                            boost::bind(&TargetLocate::doneCallback, this, _1, _2));
      
      if (targetFrame_ == "/map") {
        geometry_msgs::Pose navi;
        navi.position.x = base_point.point.x;
        navi.position.y = base_point.point.y;
        navi.position.z = 0.0;
        navi.orientation.w = rotation.w;
        navi.orientation.x = rotation.x;
        navi.orientation.y = rotation.y;
        navi.orientation.z = rotation.z;
        naviPointPublisher_.publish(navi);
        FLAG_pub_obj_pos = false;
      }
      
    }
  }
}


int main (int argc, char** argv) {
  ros::init(argc, argv, "target_localization");
  ros::NodeHandle nh("~");
  target_localization::TargetLocate localizer(nh, argc, argv);
  while(ros::ok()) {
    localizer.transformTarget();
    ros::spinOnce();
  }
}


