/*
    Date: 2019/03/13
    Author: Xu Yucheng 
    Abstract: object detection with yolo, 3D-position predict with PCL
*/
#define PI 3.1415926

// common headers
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
#include <geometry_msgs/PointStamped.h>

// user-defined ROS message type
#include <robot_control_msgs/Mission.h>
#include <robot_control_msgs/Feedback.h>

// using namespace std;
// using namespace pcl;
// using namespace cv;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_astra (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base (new pcl::PointCloud<pcl::PointXYZ>);
unsigned char floatBuffer[4];
    
bool FLAG_found_object = false;
bool FLAG_adjust_robot = false;
bool FLAG_sub_pcl_data = false;
bool FLAG_pub_obj_pos = false;

int count_detect = 0;
const int camera_width = 640;
const int camera_height = 480;
int object_x = 0;
int object_y = 0;
int origin_index = 0;
int valid_index = 0;

class object_detection {
private:
	// ROS parameters
	std::string sub_bbox_topic_name;
	std::string sub_pcl_topic_name;
	std::string sub_image_raw_topic_name;
	std::string sub_control_topic_name;
	
	std::string pub_to_control_topic_name;
	std::string pub_tf_pcl_topic_name;

	// ROS subscriber & publisher
	ros::Subscriber bbox_sub;
	ros::Subscriber pcl_sub;
	ros::Subscriber img_sub;
	ros::Subscriber control_sub;

	ros::Publisher obj_pub;
	ros::Publisher pcl_pub;

	// parameters
	const std::string CAMERA_DEPTH_FRAME = "astra_depth_frame";
	const std::string TURTLEBOT_BASE_FRAME = "base_link";

	// 定义待抓取的物体的名称，初始化为None
	std::string object_name = "none";
    
	int find_near_valid(int idx) {   
		double temp_min = 9999999999;
		int return_idx = idx;
		int obj_row = idx/camera_width;
		int obj_col = idx%camera_width;

		for (int row=0; row<camera_height; row++) {
			for (int col=0; col<camera_width; col++) {
				if (!std::isnan(cloud_astra->points[row*camera_width+col].x) &&
						!std::isnan(cloud_astra->points[row*camera_width+col].y) &&
						!std::isnan(cloud_astra->points[row*camera_width+col].z)) {
					double dis = (row-obj_row)*(row-obj_row) + (col-obj_col)*(col-obj_col);
					if (dis < temp_min) {
						return_idx = row*camera_width + col;
						temp_min = dis;
					}
				}
			}
		}
		valid_index = return_idx;
		return return_idx;
	}

	void controlCallback(const robot_control_msgs::Mission msg) {
		if (msg.action == "locate") {
			// 从control节点处获得目标物体的名称
			if (msg.target == "object") {
				object_name = msg.attributes.object.name;
				//先开启点云处理，等待2s稳定点云数据后再开始图像处理
				object_x = msg.attributes.vision.pixel_coords.pixel_x;
				object_y = msg.attributes.vision.pixel_coords.pixel_y;
				std::cout << "Received target object: " << object_name << ", pixel coordinates: " << "(" << object_x << "," << object_y << ")" << std::endl;
				FLAG_sub_pcl_data = true;
			}
		}
	}

	void imageCallback (const sensor_msgs::ImageConstPtr& msg) {
		if (!cloud_astra->empty()) {
			int ori_row = origin_index / camera_width;
			int ori_col = origin_index % camera_width;
			int val_row = valid_index / camera_width;
			int val_col = valid_index % camera_width;
			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			cv::Mat img = cv_ptr->image;
			cv::Point ori(ori_col, ori_row);
			cv::Point val(val_col, val_row);

			circle(img, ori, 3, cv::Scalar(0,0,255), -1);
			circle(img, val, 3, cv::Scalar(255,0,0), -1);

			cv::imshow("object_detection", img);
			cv::waitKey(10);
		}
	}

	void pclCallback(sensor_msgs::PointCloud2 msg) {
		// 机器人到达抓取位置之后开始监听点云变换
		if (FLAG_sub_pcl_data == true) {
			// 储存原始点云数据
			pcl::fromROSMsg (msg, *cloud_astra);
		}
	}

	void get_object_position()
	{
		if (cloud_astra->empty()) {
			ROS_INFO ("Waiting for pcl transform");
			sleep (2);
		}
		else {
			geometry_msgs::PointStamped cam_pos;
			geometry_msgs::PointStamped base_point;
			cam_pos.header.frame_id = "/astra_depth_optical_frame";
			origin_index = object_y * camera_width + object_x;
			robot_control_msgs::Feedback msg;
			tf::TransformListener pListener;

			if (std::isnan(cloud_astra->points[origin_index].x) || 
					std::isnan(cloud_astra->points[origin_index].y) ||
					std::isnan(cloud_astra->points[origin_index].z)) {  
				ROS_INFO("Fix invalid point cloud, Finding the nearest valid point");
				int new_index = find_near_valid(origin_index);
				cam_pos.header.stamp = ros::Time(0);
				cam_pos.point.x = cloud_astra->points[new_index].x;
				cam_pos.point.y = cloud_astra->points[new_index].y;
				cam_pos.point.z = cloud_astra->points[new_index].z;

				try {
					ROS_INFO("Listening for the tf transform");
					pListener.waitForTransform("/astra_depth_optical_frame", "/base_link", ros::Time(0), ros::Duration(3.0));
					pListener.transformPoint("/base_link", cam_pos, base_point);
					ROS_INFO("cam_point: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
									cam_pos.point.x, cam_pos.point.y, cam_pos.point.z,
									base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
				}
				catch(tf::TransformException& ex) {
					ROS_ERROR("Received an exception trying to transform a point from \"astra_depth_optical_frame\" to \"base_link\": %s", ex.what());
				}
			}
			else {
				ROS_INFO("Congraz! Current point is valid");
				cam_pos.header.stamp = ros::Time(0);
				cam_pos.point.x = cloud_astra->points[origin_index].x;
				cam_pos.point.y = cloud_astra->points[origin_index].y;
				cam_pos.point.z = cloud_astra->points[origin_index].z;
				try {
					ROS_INFO("Looking for transform");
					pListener.waitForTransform("/astra_depth_optical_frame", "/base_link", ros::Time(0), ros::Duration(3.0));
					pListener.transformPoint("/base_link", cam_pos, base_point);
					ROS_INFO("cam_point: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
									cam_pos.point.x, cam_pos.point.y, cam_pos.point.z,
									base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
				}
				catch(tf::TransformException& ex) {
					ROS_ERROR("Received an exception trying to transform a point from \"astra_depth_optical_frame\" to \"base_link\": %s", ex.what());
				}
			}
			if (!FLAG_pub_obj_pos) {
				// 控制发布器只发布一次消息
				msg.action = "locate";
				msg.target = "object";
				msg.mission_state = "success";
				msg.results.vision.header.frame_id = "/base_link";
				msg.results.vision.space_coords.x = base_point.point.x;
				msg.results.vision.space_coords.y = base_point.point.y;
				msg.results.vision.space_coords.z = base_point.point.z;
				FLAG_pub_obj_pos = true;
			}
		}
	}

public:
	int run (int argc, char** argv) {
		ROS_INFO("----------INIT----------");
		ros::init (argc, argv, "object_detection");
		ros::NodeHandle nh;
		ROS_INFO("----Waiting for image----");

		nh.param<std::string>("sub_pcl_topic_name",        sub_pcl_topic_name,        "/astra/depth/points");
		nh.param<std::string>("sub_image_raw_topic_name",  sub_image_raw_topic_name,  "/astra/rgb/image_raw");
		nh.param<std::string>("sub_control_topic_name",    sub_control_topic_name,    "/control_to_vision");

		nh.param<std::string>("pub_to_control_topic_name", pub_to_control_topic_name, "/vision_to_control");
		nh.param<std::string>("pub_tf_pcl_topic_name",     pub_tf_pcl_topic_name,     "/base_link_pcl");

		pcl_sub = nh.subscribe(sub_pcl_topic_name, 1, &object_detection::pclCallback, this);
		img_sub = nh.subscribe(sub_image_raw_topic_name, 1, &object_detection::imageCallback, this);
		control_sub = nh.subscribe(sub_control_topic_name, 1, &object_detection::controlCallback, this);
		
		obj_pub = nh.advertise<robot_control_msgs::Feedback>(pub_to_control_topic_name, 1);
		pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(pub_tf_pcl_topic_name, 1);
		
		std::cout << "Receving message from topics: " << std::endl;
		std::cout << "--------------------------" << std::endl;
		std::cout << "\t" << sub_bbox_topic_name << std::endl;
		std::cout << "\t" << sub_pcl_topic_name << std::endl;
		std::cout << "\t" << sub_control_topic_name << std::endl;
		std::cout << "\t" << sub_image_raw_topic_name << std::endl;
		std::cout << "--------------------------" << std::endl;
		std::cout << "Publishing message to topics: " << std::endl;
		std::cout << "--------------------------" << std::endl;
		std::cout << "\t" << pub_to_control_topic_name << std::endl;
		std::cout << "\t" << pub_tf_pcl_topic_name << std::endl;
		std::cout << "--------------------------" << std::endl;

		while (ros::ok()) {
			get_object_position();
			ros::spinOnce();
		}
	}

};

int main(int argc, char** argv) {
	object_detection detector;
	return detector.run(argc, argv);
}

