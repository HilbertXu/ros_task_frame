#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Author: Hilbert Xu
    Date: 10th July 2020
    Abstract: Base class for robot control nodes
"""
# Basic Modules
import os
import sys
import time
import rospy
import roslib
from sound_play.libsoundplay import SoundClient

# Basic class for task process control
from control_base import BasicControl, INITIAL, ONGOING, FAIL, FINISH

# User-defined ros message
from robot_control_msgs.msg import Mission
from robot_control_msgs.msg import Attributes

class GraspControlDemo(BasicControl):
    def __init__(self):
        super(GraspControlDemo, self).__init__()
        rospy.on_shutdown(self._cleanup)
        # Soundplay 参数
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.speaker = SoundClient(blocking=True)
        rospy.sleep(1)
        self.speaker.stopAll()
        rospy.sleep(1)
        """
            在本Demo中
            机器人移动到目标地点后对带抓取物体进行第一次观测，对物体进行三维空间中的定位
            然后逼近待抓取物体，对物体进行第二次三维空间中的定位
            然后再将定位后的物体三维坐标(/base_link下)发送给底盘控制器
            控制器控制机器人底盘移动到合适的抓取位置
            然后进行抓取尝试
        """

        # Add task-specific FLAGS
        self._FLAG_DETECT   = INITIAL
        self._FLAG_LOCATE   = INITIAL
        self._FLAG_SPEAKING = INITIAL

        self.main_loop()
    
    def _pub_vision(self, action, target, attributes=Attributes()):
        msg = Mission()
        msg.action = action
        msg.target = target
        msg.attributes = attributes
        # Change the state of FLAGS
        if action == "detect" and self._FLAG_DETECT != ONGOING:
            self._publish_message(self._control_pub_vision, msg)
            self._FLAG_DETECT = ONGOING
        
        if action == "stop_detect" and self._FLAG_DETECT == FINISH:
            self._publish_message(self._control_pub_vision, msg)
        
        if action == "locate" and self._FLAG_LOCATE != ONGOING:
            self._publish_message(self._control_pub_vision, msg)
            self._FLAG_LOCATE = ONGOING

    def _pub_speech(self, action, target, attributes=Attributes()):
        msg = Mission()
        msg.action = action
        msg.target = target
        msg.attributes = attributes
        # Change the state of FLAGS
        if action == "speak" and self._FLAG_SPEAKING != ONGOING:
            self._publish_message(self._control_pub_speech, msg)
            self._FLAG_SPEAKING = ONGOING
    
    def _vision_callback(self, msg):
        if msg.action == "detect":
            if msg.target == "object":
                if msg.mission_state == "success":
                    self._object_pixel_coordinates = msg.results.vision.pixel_coords
                    self._FLAG_DETECT = FINISH
                elif msg.mission_state == "fail":
                    self._FLAG_DETECT = FAIL

        elif msg.action == "locate":
            if msg.target == "object":
                if msg.mission_state == "success":
                    self._object_3d_coordinates = msg.results.vision.space_coords
                    self._FLAG_LOCATE = FINISH
                elif msg.mission_state == "fail":
                    self._FLAG_LOCATE = FAIL
        else:
            rospy.loginfo("Ooops! Wrong message was sent to {}".format(self._sub_vision_back_topic_name))
        
    def _speech_callback(self, msg):
        if msg.mission.action == "speech":
            if msg.mission_state == "success":
                self._FLAG_SPEAKING = FINISH
            elif msg.mission_state == "fail":
                self._FLAG_SPEAKING = FAIL
        else:
            rospy.loginfo("Ooops! Wrong message was sent to {}".format(self._sub_speech_back_topic_name))

    
    def main_loop(self):
        # Navigate to target location
        attrs = self._seal_attribute_message(location="diningtable")
        self._pub_navigate("navigate", "location", attrs)
        # Detect the target object
        while (True):
            if self._FLAG_NAVIGATE == FINISH:
                rospy.loginfo("I have reached the {}".format("dining table"))
                self.speaker.say("I have reached the {}".format("dining table"), self.voice)
                """
                    @TODO
                    加上语音节点，让机器人说出来这句话
                    # Example
                    self._pub_speech("speech", "speak", "I have reached the {}".format("dining table"))
                    while (True):
                        if self._FLAGS_SPEAKING == FINISH:
                            break
                """
                break

        # Launch Darknet for detecting the target object
        os.system("gnome-terminal -x bash -c 'roslaunch darknet_ros darknet_ros.launch'")
        rospy.sleep(4)
        attrs = self._seal_attribute_message(object_name="cup")
        self._pub_vision("detect", "object", attrs)
        while (True):
            if self._FLAG_DETECT == FINISH:
                self.speaker.say("I have found the target object {}".format("cup"), self.voice)
                rospy.loginfo("I have found the target object {}".format("cup"))
                rospy.loginfo("Pixel Coordinates: ({}, {})".format(self._object_pixel_coordinates.pixel_x, self._object_pixel_coordinates.pixel_y))
                self._pub_vision("stop_detect", "object", attrs)
                rospy.sleep(1)
                break

        # Locate the target object in 3D space using point cloud and TransForm
        os.system("gnome-terminal -x bash -c 'rosrun robot_vision_core object_localization'")
        rospy.sleep(3)
        attrs = self._seal_attribute_message(object_name="cup", pixel_coords=self._object_pixel_coordinates)
        self._pub_vision("locate", "object", attrs)
        while (True):
            if self._FLAG_LOCATE == FINISH:
                self.speaker.say("I have located the target object {}".format("cup"), self.voice)
                rospy.loginfo("I have located the target object {}".format("cup"))
                rospy.loginfo("Space Coordinates: ({},{})".format(self._object_3d_coordinates.x,self._object_3d_coordinates.y))
                os.system("rosnode kill /object_localization")
                break

        # Approach to the target object
        os.system("gnome-terminal -x bash -c 'rosrun robot_navigation_core odom_adjust_for_grasp.py'")
        rospy.sleep(4)
        attrs = self._seal_attribute_message(object_name="cup", space_coords=self._object_3d_coordinates)
        self._pub_navigate("approach", "object", attrs)
        while (True):
            if self._FLAG_APPROACH == FINISH:
                rospy.loginfo("I have reached the target object {}".format("cup"))
                rospy.loginfo("Now I will relocate the target object for accuracy")
                self.speaker.say("I have reached the target object {}".format("cup"), self.voice)
                self.speaker.say("Now I will relocate the target object for accuracy", self.voice)
                break
        
        # Re-detect the target object
        attrs = self._seal_attribute_message(object_name="cup")
        self._pub_vision("detect", "object", attrs)
        while (True):
            if self._FLAG_DETECT == FINISH:
                rospy.loginfo("I have found the target object again")
                rospy.loginfo("New Pixel Coordinates: {}".format(self._object_pixel_coordinates))
                self.speaker.say("I have found the target object again", self.voice)
                break
        
        # Re-locate the target object in 3D space
        os.system("gnome-terminal -x bash -c 'rosrun robot_vision_core object_localization'")
        rospy.sleep(2)
        attrs = self._seal_attribute_message(object_name="cup", pixel_coords=self._object_pixel_coordinates)
        self._pub_vision("locate", "object", attrs)
        while (True):
            if self._FLAG_LOCATE == FINISH:
                rospy.loginfo("I have located the target object {} again".format("cup"))
                rospy.loginfo("New Space Coordinates: {}".format(self._object_3d_coordinates))
                self.speaker.say("I have located the target object {} again".format("cup"), self.voice)
                os.system("rosnode kill /object_localization")
                self._pub_vision("stop_detect", "object", attrs)
                break
        
        # Re-approach the target object
        attrs = self._seal_attribute_message(object_name="cup", 
                                             pixel_coords=self._object_pixel_coordinates,
                                             space_coords=self._object_3d_coordinates,
                                            )
        self._pub_navigate("approach", "object", attrs)
        while (True):
            if self._FLAG_APPROACH == FINISH:
                rospy.loginfo("I have reached the target object {}".format("cup"))
                self.speaker.say("I have reached the target object {}".format("cup"), self.voice)
                break
        
        # Try to grasp the target object
        os.system("gnome-terminal -x bash -c 'rosrun turtlebot_arm_moveit_demos grasp_target_object.py'")
        rospy.sleep(4)
        self._pub_arm("grasp", "object", attrs)
        while (True):
            if self._FLAG_GRASP == FINISH:
                rospy.loginfo("I have grasped the target object {}".format("cup"))
                rospy.loginfo("Now I will bring it to the {}".format("livingtable"))
                self.speaker.say("I have grasped the target object {}".format("cup"), self.voice)
                self.speaker.say("Now I will go back to the start point", self.voice)
                break
        
        # Back to the observation point
        self._pub_navigate("approach", "start_point")
        while (True):
            if self._FLAG_APPROACH == FINISH:
                self.speaker.say("I have back to the start point", self.voice)
                rospy.loginfo("I have back to the start point")
                self.speaker.say("Now i will go back to the operator", self.voice)
                break

        # Navigate to the next location
        attrs = self._seal_attribute_message(location="livingtable")
        self._pub_navigate("navigate", "location", attrs)
        while (True):
            if self._FLAG_NAVIGATE == FINISH:
                rospy.loginfo("I have arrived at the {}".format("operator"))
                rospy.loginfo("Now I will release the object")
                self.speaker.say("I have arrived at the {}".format("living table"), self.voice)
                self.speaker.say("Now I will release the object", self.voice)
                break

        # Release the object
        attrs = self._seal_attribute_message(object_name="cup")
        self._pub_arm("release", "object", attrs)
        while (True):
            if self._FLAG_RELEASE == FINISH:
                rospy.loginfo("I have released the object")
                self.speaker.say("I have released the object", self.voice)
                break     
    #     # Navigate to target location
    #     attrs = self._seal_attribute_message(location="diningtable")
    #     self._pub_navigate("navigate", "location", attrs)
    #     # Detect the target object
    #     while (True):
    #         if self._FLAG_NAVIGATE == FINISH:
    #             rospy.loginfo("I have reached the {}".format("dining table"))
    #             """
    #                 @TODO
    #                 加上语音节点，让机器人说出来这句话
    #                 # Example
    #                 self._pub_speech("speech", "speak", "I have reached the {}".format("dining table"))
    #                 while (True):
    #                     if self._FLAGS_SPEAKING == FINISH:
    #                         break
    #             """
    #             break

    #     # Launch Darknet for detecting the target object
    #     os.system("gnome-terminal -x bash -c 'roslaunch darknet_ros darknet_ros.launch'")
    #     rospy.sleep(3)
    #     attrs = self._seal_attribute_message(object_name="cup")
    #     self._pub_vision("detect", "object", attrs)
    #     while (True):
    #         if self._FLAG_DETECT == FINISH:
    #             rospy.loginfo("I have found the target object {}".format("cup"))
    #             rospy.loginfo("Pixel Coordinates: ({}, {})".format(self._object_pixel_coordinates.pixel_x, self._object_pixel_coordinates.pixel_y))
    #             self._pub_vision("stop_detect", "object", attrs)
    #             rospy.sleep(1)
    #             break

    #     # Locate the target object in 3D space using point cloud and TransForm
    #     os.system("gnome-terminal -x bash -c 'rosrun robot_vision_core object_localization'")
    #     rospy.sleep(3)
    #     attrs = self._seal_attribute_message(object_name="cup", pixel_coords=self._object_pixel_coordinates)
    #     self._pub_vision("locate", "object", attrs)
    #     while (True):
    #         if self._FLAG_LOCATE == FINISH:
    #             rospy.loginfo("I have located the target object {}".format("cup"))
    #             rospy.loginfo("Space Coordinates: ({},{})".format(self._object_3d_coordinates.x,self._object_3d_coordinates.y))
    #             os.system("rosnode kill /object_localization")
    #             break

    #     # Approach to the target object
    #     os.system("gnome-terminal -x bash -c 'rosrun robot_navigation_core odom_adjust_for_grasp.py'")
    #     rospy.sleep(4)
    #     attrs = self._seal_attribute_message(object_name="cup", space_coords=self._object_3d_coordinates)
    #     self._pub_navigate("approach", "object", attrs)
    #     while (True):
    #         if self._FLAG_APPROACH == FINISH:
    #             rospy.loginfo("I have reached the target object {}".format("cup"))
    #             rospy.loginfo("Now I will relocate the target object for accuracy")
    #             break
        
    #     # Re-detect the target object
    #     attrs = self._seal_attribute_message(object_name="cup")
    #     self._pub_vision("detect", "object", attrs)
    #     while (True):
    #         if self._FLAG_DETECT == FINISH:
    #             rospy.loginfo("I have found the target object again")
    #             rospy.loginfo("New Pixel Coordinates: {}".format(self._object_pixel_coordinates))
    #             break
        
    #     # Re-locate the target object in 3D space
    #     os.system("gnome-terminal -x bash -c 'rosrun robot_vision_core object_localization'")
    #     rospy.sleep(2)
    #     attrs = self._seal_attribute_message(object_name="cup", pixel_coords=self._object_pixel_coordinates)
    #     self._pub_vision("locate", "object", attrs)
    #     while (True):
    #         if self._FLAG_LOCATE == FINISH:
    #             rospy.loginfo("I have located the target object {} again".format("cup"))
    #             rospy.loginfo("New Space Coordinates: {}".format(self._object_3d_coordinates))
    #             os.system("rosnode kill /object_localization")
    #             self._pub_vision("stop_detect", "object", attrs)
    #             break
        
    #     # Re-approach the target object
    #     attrs = self._seal_attribute_message(object_name="cup", 
    #                                          pixel_coords=self._object_pixel_coordinates,
    #                                          space_coords=self._object_3d_coordinates,
    #                                         )
    #     self._pub_navigate("approach", "object", attrs)
    #     while (True):
    #         if self._FLAG_APPROACH == FINISH:
    #             rospy.loginfo("I have reached the target object {}".format("cup"))
    #             break
        
    #     # Try to grasp the target object
    #     os.system("gnome-terminal -x bash -c 'rosrun turtlebot_arm_moveit_demos grasp_target_object.py'")
    #     rospy.sleep(4)
    #     self._pub_arm("grasp", "object", attrs)
    #     while (True):
    #         if self._FLAG_GRASP == FINISH:
    #             rospy.loginfo("I have grasped the target object {}".format("cup"))
    #             rospy.loginfo("Now I will bring it to the {}".format("livingtable"))
    #             break
        
    #     # Back to the observation point
    #     self._pub_navigate("approach", "start_point")
    #     while (True):
    #         if self._FLAG_APPROACH == FINISH:
    #             rospy.loginfo("I have back to the start point")
    #             break

    #     # Navigate to the next location
    #     attrs = self._seal_attribute_message(location="livingtable")
    #     self._pub_navigate("navigate", "location", attrs)
    #     while (True):
    #         if self._FLAG_NAVIGATE == FINISH:
    #             rospy.loginfo("I have arrived at the {}".format("living table"))
    #             rospy.loginfo("Now I will release the object")
    #             break

    #     # Release the object
    #     attrs = self._seal_attribute_message(object_name="cup")
    #     self._pub_arm("release", "object", attrs)
    #     while (True):
    #         if self._FLAG_RELEASE == FINISH:
    #             rospy.loginfo("I have released the object")
    #             rospy.loginfo("Now i will go back to the operator")
    #             break                


if __name__ == "__main__":
    rospy.init_node("grasp_control_demo", disable_signals=True)
    controller = GraspControlDemo()
    rospy.spin()
    controller.main_loop()
    
