#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Author: Hilbert Xu
    Date: 10th July 2020
    Abstract: Base class for robot control nodes
"""

import roslib
import rospy
from std_msgs.msg import Header
from robot_control_msgs.msg import Mission
from robot_control_msgs.msg import Attributes
from robot_control_msgs.msg import PixelCoords, SpaceCoords
from robot_control_msgs.msg import Feedback
# 定义表示任务状态的变量
INITIAL = 0
ONGOING = 1
FAIL    = 2
FINISH  = 3

class BasicControl(object):
    """
        Class for grasp control demo
        机器人通过导航运动到dining table，然后开始抓取环节，
        完成抓取后机器人导航到living table并将抓取到的物体放置在living table上
    """
    def __init__(self):
        rospy.loginfo("Grasp Control Demo online!")

        # Def publisher
        """
            Control Node提供了其他所有节点的控制
            Arm: 通过/control_to_arm话题发布机械臂控制命令，控制机械臂进行抓取或者放置，同时控制机械臂节点的开始和结束
            vision: 通过/control_to_vision话题发布识别对象和目标，控制识别过程的开始，重复，结束
            Speech: 通过/control_to_speech话题控制录音节点的开启与关闭，同时控制sound_play节点播放声音
            Navigation: 通过/control_to_nav话题发布导航目标点
        """
        self._control_pub_arm        = None
        self._control_pub_vision      = None
        self._control_pub_speech     = None
        self._control_pub_navigation = None

        # Def Subscriber
        """
            Control Node接收来自其他所有节点的反馈消息
            Arm: 是否完成抓取或放置
            vision: 是否识别到指定对象
            Speech: 录音节点当前状态，是否成功播放声音
            Navigation: 是否到达导航目标点
        """
        self._control_sub_arm        = None
        self._control_sub_vision      = None
        self._control_sub_speech     = None
        self._control_sub_navigation = None

        # Initialize parameters of base class
        self._init_params()

    def _init_params(self):
        # Target sets
        """
            导航，图像，物体，人物，任务目标集合
            在本demo中固定为去dining table抓取cup然后放置到living table
            留出与语音结合的接口
        """
        self._target_location = []
        self._target_object   = []
        self._target_person   = []
        self._target_gesture  = []
        self._target_gender   = []
        self._target_pose     = []
        self._target_action   = []

        # Flags of Mission process
        """
            指示当前任务状态的变量
            每个任务目前设置了可能的四个状态:
            INITIAL, ONGOING, FAIL, FINISH
            在本demo中，将抓取任务分为导航，逼近，抓取，放置四个部分
        """
        self._FLAG_NAVIGATE = INITIAL
        self._FLAG_APPROACH = INITIAL
        self._FLAG_FOLLOW   = INITIAL
        self._FLAG_GRASP    = INITIAL
        self._FLAG_RELEASE  = INITIAL

        # Set up topics name
        self._sub_vision_back_topic_name  = rospy.get_param("sub_vision_back_topic_name", "/vision_to_control")
        self._sub_arm_back_topic_name    = rospy.get_param("sub_arm_back_topic_name", "/arm_to_control")
        self._sub_nav_back_topic_name    = rospy.get_param("sub_nav_back_topic_name", "/nav_to_control")
        self._sub_speech_back_topic_name = rospy.get_param("sub_speech_back_topic_name", "/speech_to_control")

        self._pub_to_arm_topic_name    = rospy.get_param("pub_to_arm_topic_name", "/control_to_arm")
        self._pub_to_vision_topic_name  = rospy.get_param("pub_to_vision_topic_name", "/control_to_vision")
        self._pub_to_nav_topic_name    = rospy.get_param("pub_to_nav_topic_name", "/control_to_nav")
        self._pub_to_speech_topic_name = rospy.get_param("pub_to_speech_topic_name", "/control_to_speech") 

        # Set up Subscribers and Publishers
        self._control_sub_arm        = rospy.Subscriber(self._sub_arm_back_topic_name, Feedback, self._arm_callback)
        self._control_sub_vision      = rospy.Subscriber(self._sub_vision_back_topic_name, Feedback, self._vision_callback)
        self._control_sub_speech     = rospy.Subscriber(self._sub_speech_back_topic_name, Feedback, self._speech_callback)
        self._control_sub_navigation = rospy.Subscriber(self._sub_nav_back_topic_name, Feedback, self._nav_callback)

        self._control_pub_arm        = rospy.Publisher(self._pub_to_arm_topic_name, Mission, queue_size=1)
        self._control_pub_vision      = rospy.Publisher(self._pub_to_vision_topic_name, Mission, queue_size=1)
        self._control_pub_speech     = rospy.Publisher(self._pub_to_speech_topic_name, Mission, queue_size=1)
        self._control_pub_navigation = rospy.Publisher(self._pub_to_nav_topic_name, Mission, queue_size=1)

    def _cleanup(self):
        rospy.signal_shutdown("See You Next Time~")

    def _seal_attribute_message(
        self,
        attrs_msg=None,
        human_name='',
        human_gender='',
        human_gesture='',
        object_name='',
        object_color='',
        location='',
        header=Header(),
        pixel_coords=PixelCoords(),
        space_coords=SpaceCoords(),
        sentence=''
        ):
        """
            Seal the Attributes msg of Mission
        """
        if attrs_msg is None:
            attrs_msg = Attributes()
        attrs_msg.human.name          = human_name
        attrs_msg.human.gender        = human_gender
        attrs_msg.human.gesture       = human_gesture
        attrs_msg.object.name         = object_name
        attrs_msg.object.color        = object_color
        attrs_msg.navi.location       = location
        attrs_msg.vision.header       = header
        attrs_msg.vision.pixel_coords = pixel_coords
        attrs_msg.vision.space_coords = space_coords
        attrs_msg.speech.sentence     = sentence

        return attrs_msg


    def _publish_message(self, puber, msg):
        """
            创建一个用于消息发布的接口
        """
        puber.publish(msg)

    def _pub_arm(self, action, target, attributes=Attributes()):
        msg = Mission()
        msg.action = action
        msg.target = target
        msg.attributes = attributes
        # Change the state of FLAGS
        if action == "grasp" and self._FLAG_GRASP != ONGOING:
            self._publish_message(self._control_pub_arm, msg)
            self._FLAG_GRASP = ONGOING
        elif action == "release" and self._FLAG_RELEASE != ONGOING:
            self._publish_message(self._control_pub_arm, msg)
            self._FLAG_RELEASE = ONGOING
        else:
            rospy.loginfo("Ooops! {} is used to sent incorrect message".format(self._pub_to_arm_topic_name))
    
    def _pub_navigate(self, action, target, attributes=Attributes()):
        msg = Mission()
        msg.action = action
        msg.target = target
        msg.attributes = attributes
        # Change the state of FLAGS
        if action == "navigate" and self._FLAG_NAVIGATE != ONGOING:
            self._publish_message(self._control_pub_navigation, msg)
            self._FLAG_NAVIGATE = ONGOING
        elif action == "approach" and self._FLAG_APPROACH != ONGOING:
            self._publish_message(self._control_pub_navigation, msg)
            self._FLAG_APPROACH = ONGOING
        elif action == "follow" and self._FLAG_FOLLOW != ONGOING:
            self._publish_message(self._control_pub_navigation, msg)
            self._FLAG_FOLLOW = ONGOING
        else:
            rospy.loginfo("Ooops! {} is used to sent incorrect message".format(self._pub_to_nav_topic_name))
    
    """
        实际任务中图像模块和语音模块的判断流程会比较复杂
        故在基类中留出接口，在子类中实现判断
    """
    def _pub_vision(self, action, target, attributes=Attributes()):
        raise NotImplementedError()

    def _pub_speech(self, action, target, attributes=Attributes()):
        raise NotImplementedError()

    # Callback Functions
    def _nav_callback(self, msg):
        if msg.action == "navigate":
            if msg.mission_state == "success":
                self._FLAG_NAVIGATE = FINISH
            elif msg.mission_state == "fail":
                self._FLAG_NAVIGATE = FAIL

        elif msg.action == "approach":
            if msg.mission_state == "success":
                self._FLAG_APPROACH = FINISH
            elif msg.mission_state == "fail":
                self._FLAG_APPROACH = FAIL

        elif msg.action == "follow":
            if msg.mission_state == "success":
                self._FLAG_FOLLOW = FINISH
            elif msg.mission_state == "fail":
                self._FLAG_FOLLOW = FAIL
        else:
            rospy.loginfo("Ooops! Wrong message was sent to {}".format(self._sub_arm_back_topic_name))
    
    def _arm_callback(self, msg):
        if msg.action == "grasp":
            if msg.mission_state == "success":
                self._FLAG_GRASP = FINISH
            elif msg.mission_state == "fail":
                self._FLAG_GRASP = FAIL

        elif msg.action == "release":
            if msg.mission_state == "success":
                self._FLAG_RELEASE = FINISH
            elif msg.mission_state == "fail":
                self._FLAG_RELEASE = FAIL
        else:
            rospy.loginfo("Ooops! Wrong message was sent to {}".format(self._sub_nav_back_topic_name))
    
    """
        实际任务中图像模块和语音模块的判断流程会比较复杂
        故在基类中留出接口，在子类中实现判断
    """
    def _vision_callback(self, msg):
        raise NotImplementedError

    def _speech_callback(self, msg):
        raise NotImplementedError

    # 任务主循环函数，具体实现根据不同任务来调整
    def main_loop(self):
        """
            如果用于比赛，建议为每个任务增加计时器
            一定时间后跳出当前任务环节的循环，进入下一个环节
            确保机器人在某个环节中失败后可以完成整个比赛流程
        """
        raise NotImplementedError


        