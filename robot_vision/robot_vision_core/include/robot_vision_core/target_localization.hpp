/*
 * target_localization.hpp
 * 
 *  Created on: Aug 26th, 2020
 *      Author: Hilbert Xu
 *   Institute: Mustar Robot
 *   使用前修改头部电机速度为0.1
 */

#define PI 3.1415926

// C++ Headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <cmath>
#include <sstream>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>

// Robot control message type
#include <robot_vision_msgs/PixelPoint.h>
#include <robot_vision_msgs/SpacePoint.h>
#include <robot_control_msgs/Mission.h>
#include <robot_control_msgs/Feedback.h>

// Move robot action server
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_navigation_msgs/MoveRobotAction.h>

typedef actionlib::SimpleActionClient<robot_navigation_msgs::MoveRobotAction> MoveRobotActionClient;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAstra (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBase (new pcl::PointCloud<pcl::PointXYZ>);
unsigned char floatBuffer[4];

namespace target_localization {
  class TargetLocate {
  public:
    //! Constructor
    explicit TargetLocate(ros::NodeHandle nh, int argc, char** argv);

    //! Destrcutor
    ~TargetLocate();

    //! transform target point from source frame to target frame
    void transformTarget ();
  private:
    // Flags
    bool FLAG_under_control = false;
    bool FLAG_sub_pcl_data = false;
    bool FLAG_pub_obj_pos = true;

    // Parameters
    int cameraWidth_ = 0;
    int cameraHeight_ = 0;
    int object_x = 0;
    int object_y = 0;
    int originIndex_ = 0;
    int validIndex_ = 0;

    // source frame and target frame for tf
    std::string sourceFrame_;
    std::string targetFrame_;

    // Robot pose
    geometry_msgs::Quaternion rotation;

    // ROS NodeHandle
    ros::NodeHandle nodeHandle_;

    // ROS Publishers & Subscribers
    ros::Subscriber pclSubscriber_;
    ros::Subscriber imageSubscriber_;
    ros::Subscriber controlSubscriber_;
    ros::Subscriber cameraInfoSubscriber_;
    ros::Subscriber pixelPointSubscriber_;

    ros::Publisher controlPublisher_;
    ros::Publisher spacePointPublisher_;
    ros::Publisher naviPointPublisher_;

    // Move robot actionlib
    actionlib::SimpleActionClient<robot_navigation_msgs::MoveRobotAction> actionClient_;

    // Initial function
    void init();

    // actionlib done callback for getting robot pose
    void doneCallback(const actionlib::SimpleClientGoalState &state, const robot_navigation_msgs::MoveRobotResultConstPtr &result);

    // Find the nearest valid point if needed
    int findNearValid (int idx);

    // Target pixel point callback
    void pixelPointCallback (const robot_vision_msgs::PixelPointConstPtr &msg);

    // Camera info callback 
    void cameraInfoCallback (const sensor_msgs::CameraInfoConstPtr &msg);

    // Control callback
    void controlCallback (const robot_control_msgs::MissionConstPtr &msg);

    // Image callback
    void imageCallback (const sensor_msgs::ImageConstPtr &msg);

    // Point cloud callback
    void pclCallback (sensor_msgs::PointCloud2 msg);
  };
}