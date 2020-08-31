#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("/camera_top/rgb/image_raw", 1);

  cv::Mat image = cv::imread("/home/kamerider/Pictures/sb.jpg", CV_LOAD_IMAGE_COLOR);
  cv::Mat dst;
  cv::resize(image, dst, cv::Size(480, 640));
  if(image.empty()){
   printf("open error\n");
   }
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
