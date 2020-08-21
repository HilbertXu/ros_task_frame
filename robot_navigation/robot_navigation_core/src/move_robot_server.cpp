/*
 * move_robot_server.hpp
 * 
 *  Created on: 21th Aug 2020
 *      Author: Hilbert Xu
 *   Institute: Mustar Robot
 */

#include <robot_navigation_core/move_robot_server.hpp>

using namespace move_robot_server;

MoveRobotServer::MoveRobotServer(ros::NodeHandle nh, std::string name)
  :nodeHandle_(nh),
  actionServer_(nodeHandle_, name, boost::bind(&MoveRobotServer::executeCallback, this, _1), false),
  actionName_(name),
  angularSpeed_(0.25),
  linearSpeed_(0.15),
  angularTolerance_(1.0/180.0*PI)
{
    ROS_INFO("[MoveRobotServer] Waiting for transform from %s to %s", odomFrame_.c_str(), baseFrame_.c_str());
    try
    {
      pListener.waitForTransform(odomFrame_, baseFrame_, ros::Time(0), ros::Duration(1.0));
    }
    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", odomFrame_.c_str(), baseFrame_.c_str(), ex.what());
    }
    ROS_INFO("[MoveRobotServer] Initializing...");
    std::string cmdVelTopicName_;
    int cmdVelQueueSize_;
    bool cmdVelLatch_;
    nodeHandle_.param("publishers/cmd_vel/topic", cmdVelTopicName_, std::string("/cmd_vel_mux/input/navi"));
    nodeHandle_.param("publishers/cmd_vel/queue_size", cmdVelQueueSize_, 1);
    nodeHandle_.param("publishers/cmd_vel/latch", cmdVelLatch_, false);
    cmdVelPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>(cmdVelTopicName_, cmdVelQueueSize_, cmdVelLatch_);
    actionServer_.start();
}

MoveRobotServer::~MoveRobotServer(void) {
  ROS_INFO("[MoveRobotServer] Move robot server shutting down...");
}

float MoveRobotServer::quatToAngle(tf::Quaternion quat) {
  Eigen::Quaterniond quat_out(quat.w(), quat.x(), quat.y(), quat.z());
  Eigen::Vector3d eulerAngle = quat_out.matrix().eulerAngles(2,1,0);
  // eulerAngle.transpose(): yaw(z) pitch(y) roll(x)
  float zAxisAngle = eulerAngle.transpose()(0);
  return zAxisAngle;
}

float MoveRobotServer::normalizeAngle(float angle) {
  float res = angle;
  if (angle > PI) {
    res -= 2.0 * PI;
  } else if (angle < -PI) {
    res += 2.0 * PI;
  }
  return res;
}

void MoveRobotServer::turnRobot(ros::Rate loop_rate, float goalRadius, tf::Quaternion rot) {
  ROS_INFO("[MoveRobotServer] Turning robot for %f radius", goalRadius);
  
  // Set robot angular speed
  geometry_msgs::Twist cmd;
  cmd.angular.z = angularSpeed_;
  double goal_ = goalRadius;
  currAngle_ = 0;

  // Set target angle and speed according to negative/positive
  if (goalRadius < 0) {
    cmd.angular.z = -angularSpeed_;
    goal_ = -goalRadius;
  }
  // Initialize last angle and turned angle
  double lastAngle_ = quatToAngle(rot);
  double turnAngle_ = 0;

  // while loop for turning robot
  while (fabs(turnAngle_ + angularTolerance_) < fabs(goal_)) {
    // Publish feedback
    feedBack_.current_angle = lastAngle_;
    actionServer_.publishFeedback(feedBack_);

    // Check if preempted by other nodes
    if (actionServer_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", actionName_.c_str());
      // Set the action state to preempted
      actionServer_.setPreempted();
      FLAG_success = false;
      break;
    }
    // Publish message to turn robot
    cmdVelPublisher_.publish(cmd);
    loop_rate.sleep();

    // Listen current transform from /odom to /base_footprint
    transform = getOdomTransform();
    
    // Get current Z-Axis angle
    float zRotation = quatToAngle(transform.getRotation());

    // Calculate delta angle
    float deltaAngle_ = normalizeAngle(zRotation - lastAngle_);
    turnAngle_ += deltaAngle_;
    lastAngle_ = zRotation;
  }
  cmd.angular.z = 0.0;
  cmdVelPublisher_.publish(cmd);
  ros::Duration(1.0).sleep();
}

void MoveRobotServer::goForward(ros::Rate loop_rate, float distance, tf::Vector3 position) {
  ROS_INFO("[MoveRobotServer] Going forward for %f meters", distance);
  // Set robot angular speed
  geometry_msgs::Twist cmd;
  cmd.linear.x = linearSpeed_;
  double goal_ = distance;
  double start_x = position.x();
  double start_y = position.y();
  currDistance_ = 0;
  double traveledDistance_ = 0;

  // Set target angle and speed according to negative/positive
  if (distance < 0) {
    cmd.linear.x = -linearSpeed_;
    goal_ = -distance;
  }

  while (traveledDistance_ < goal_) {
    // Publish feedback
    feedBack_.current_distance = traveledDistance_;
    actionServer_.publishFeedback(feedBack_);
    
    // Check if preempted by other nodes
    if (actionServer_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("[MoveRobotServer] Preempted");
      // Set the action state to preempted
      actionServer_.setPreempted();
      FLAG_success = false;
      break;
    }
    cmdVelPublisher_.publish(cmd);
    loop_rate.sleep();
    // Get transform from odom to base
    transform = getOdomTransform();
    traveledDistance_ = sqrt(pow((transform.getOrigin().x() - start_x), 2) + pow((transform.getOrigin().y() - start_y), 2));
  }
  cmd.linear.x = 0.0;
  cmdVelPublisher_.publish(cmd);
  ros::Duration(1.0).sleep();
}

void MoveRobotServer::executeCallback(const robot_navigation_msgs::MoveRobotGoalConstPtr &goal) {
  ros::Rate loop_rate(20);
  FLAG_success = true;

  // publish info to the console for the user
  ROS_INFO("[MoveRobotServer] Executing, turning robot %f radius, go forward %f meter", goal->angle, goal->distance);

  // Get current transform from /odom to /base_footprint
  tf::StampedTransform transform = getOdomTransform();

  if (goal->angle != 0.0 && goal->distance != 0.0) {
    // 先转动机器人再移动机器人
    turnRobot(loop_rate, goal->angle, transform.getRotation());
    goForward(loop_rate, goal->distance, transform.getOrigin());
  } else if (goal->angle != 0.0) {
    turnRobot(loop_rate, goal->angle, transform.getRotation());
  } else if (goal->distance != 0.0) {
    goForward(loop_rate, goal->distance, transform.getOrigin());
  } else {
    ROS_INFO("[MoveRobotServer] Nothing to do...");
  }

  if(FLAG_success)
  {
    // uint8 SUCCEEDED=3
    if (goal->angle != 0.0 && goal->distance != 0.0) {
      result_.angle_result = "success";
      result_.distance_result = "success";
    } else if (goal->angle != 0.0) {
      result_.angle_result = "success";
    } else if (goal->distance != 0.0) {
      result_.distance_result = "success";
    }
    ROS_INFO("[MoveRobotServer] Succeeded");
    // set the action state to succeeded
    actionServer_.setSucceeded(result_);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_robot_server");
  ros::NodeHandle nh_;
  MoveRobotServer server(nh_, ros::this_node::getName());

  ros::spin();
  return 0;
}



