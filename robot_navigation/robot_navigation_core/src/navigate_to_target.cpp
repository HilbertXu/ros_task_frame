/*
 * navigate_to_target.cpp
 * 
 *  Created on: 28th Aug 2020
 *      Author: Hilbert Xu
 *   Institute: Mustar Robot
 */

#include <robot_navigation_core/navigate_to_target.hpp>

using namespace navigate_to_target;

NavigateToTarget::NavigateToTarget(ros::NodeHandle nh, int argc, char** argv): nodeHandle_(nh) {
  ROS_INFO("[Navigator] Waiting for navigation pose...");
  
  std::string mapTopicName_;
  int mapQueueSize_;

  std::string naviPoseTopicName_;
  int naviPoseQueueSize_;

  nodeHandle_.param("subscribers/map/topic", mapTopicName_, std::string("/map"));
  nodeHandle_.param("subscribers/map/queue_size", mapQueueSize_, 1);
  nodeHandle_.param("subscribers/navi_pose/topic", naviPoseTopicName_, std::string("/robot_navigation/navi_pose"));

  mapSubscriber_        = nodeHandle_.subscribe(mapTopicName_, mapQueueSize_, &NavigateToTarget::mapCallback, this);
  targetPoseSubscriber_ = nodeHandle_.subscribe(naviPoseTopicName_, naviPoseQueueSize_, &NavigateToTarget::targetPoseCallback, this);

  MoveBaseClient mClient_("move_base", true);
  move_base_msgs::MoveBaseGoal goal_;

  while (ros::ok()) {
    if (FLAG_start_navigate) {
      goal_.target_pose.header.frame_id = "/map";
      goal_.target_pose.header.stamp    = ros::Time::now();
      goal_.target_pose.pose = geometry_msgs::Pose(validPose_);
      ROS_INFO("[Navigator] Start navigating to position (%f, %f, %f), Orientation (%f, %f, %f, %f)", 
              validPose_.position.x, validPose_.position.y, validPose_.position.z,
              validPose_.orientation.x, validPose_.orientation.y, validPose_.orientation.z, validPose_.orientation.w);

      // Check if the server is activated
      while (!mClient_.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("[Navigator] Waiting for move base server...");
      }
      // Send goal to the server
      mClient_.sendGoal(goal_);
      // Watiting for the navigation result for 40 seconds
      mClient_.waitForResult(ros::Duration(40.0));
      if (mClient_.getState == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("[Navigator] Navigation task compeleted!");
      }
    }
    ros::spinOnce();
  } 
}

NavigateToTarget::~NavigateToTarget() {
  ROS_INFO("[Navigator] Shutting down...");
}

void NavigateToTarget::checkValid(geometry_msgs::Pose targetPose) {
  float target_x = targetPose.position.x;
  float target_y = targetPose.position.y;
  std::vector map_coords(worldToMap(target_x, target_y));
  
  int index = (map_coords[0] - map_.info.origin.position.x)/map_.info.resolution + (map_coords[1] - map_.info.origin.position.y)/map_.info.resolution*map_.info.width;

  int map_data = map_.data[index];

  if (map_data == -1 || map_data == 100) {
    ROS_INFO("[Navigator] Target pose is unreachable...Searching for the nearest valid pose");
    // findNearestValid
  } else {
    ROS_INFO("[Navigator] Target pose is reachable...");
    FLAG_start_navigate = true;
  }
}

std::vector<int> NavigateToTarget::worldToMap(float world_x, float world_y) {
  std::vector<int> buffer(2);
  if (world_x < map_.info.origin.position.x || world_y < map_.info.origin.position.y) {
    buffer[0] = -1;
    buffer[1] = -1;
    return buffer;
  } else {
    int map_x = int(world_x - map_.info.origin.position.x)/map_.info.resolution);
    int map_y = int(world_y - map_.info.origin.position.y)/map_.info.resolution);
    if (map_x > map_.info.width || map_y > map_.info.height) {
      buffer[0] = -1;
      buffer[0] = -1;
      return buffer;
    }
    buffer[0] = map_x;
    buffer[1] = map_y;
    return buffer;
  }
}

std::vector<int> NavigateToTarget::mapToWorld(int map_x, int map_y) {
  std::vector<int> buffer(2);
  float world_x = map_x * map_.info.resolution + map_.info.origin.position.x;
  float world_y = map_y * map_.info.resolution + map_.info.origin.position.y;
  buffer[0] = world_x;
  buffer[1] = world_y;
  return buffer;
}

void NavigateToTarget::mapCallback(const nav_msgs::OccupancyGridConstPtr &msg) {
  if (map_.data.empty()) {
    map_.info.resolution = msg->info.resolution;
    map_.info.origin     = msg->info.origin;
    map_.info.width      = msg->info.width;
    map_.info.height     = msg->info.height;
    map_.data            = msg->data;
  }
}

void NavigateToTarget::targetPoseCallback(const geometry_msgs::PoseConstPtr &msg) {
  ROS_INFO("[Navigator] Target received!");
  targetPose_.position.x = msg->position.x;
  targetPose_.position.y = msg->position.y;
  targetPose_.position.z = msg->position.z;
  targetPose_.orientation.w = msg->orientation.w;
  targetPose_.orientation.x = msg->orientation.x;
  targetPose_.orientation.y = msg->orientation.y;
  targetPose_.orientation.z = msg->orientation.z;
  // Check if current target pose is valid pose
}