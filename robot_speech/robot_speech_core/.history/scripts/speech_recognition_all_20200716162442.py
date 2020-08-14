#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 16 July, 2020
    Author: Hilbert Xu
    Abstract: Audio recording & Speech recognition
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
