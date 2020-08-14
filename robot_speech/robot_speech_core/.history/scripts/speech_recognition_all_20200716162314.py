#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 2019/03/06
    Author: Xu Yucheng
    Abstract: 记录麦克风的输入音频并保存
"""
import os
import sys
import time
import rospy
import wave
import pyaudio
import wave
from std_msgs.msg import Int8
from std_msgs.msg import String
from play_signal_sound import play_signal_sound