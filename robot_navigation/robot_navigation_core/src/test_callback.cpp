
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <std_msgs/String.h>
#include <string.h>
//actionlib头文件
#include <stdlib.h>
#include <cstdlib>
#include <vector>

using namespace std;


class test_callback {
private:
  ros::Subscriber sub1;
  ros::Subscriber sub2;
  void test(std_msgs::String::ConstPtr msg) {
    if (msg->data == "start") {
      ROS_INFO("Received message!");
      for (int i =0; i<1000000; i++) {
        std::cout << i << std::endl;
        sleep(0.1);
      }
    }
  }
public:
	int run(int argc, char** argv)
	{
		ROS_INFO ("--------INIT--------");
		ros::init (argc, argv, "test_callback");
		ros::NodeHandle nh;

    sub1 = nh.subscribe("/test1",1, &test_callback::test, this);
    sub2 = nh.subscribe("/test2",1, &test_callback::test, this);

    ros::spin();
	}
};

int main(int argc, char** argv) {
	test_callback nav;
	return nav.run(argc, argv);
}
