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

CURRENT_FOLDER_PATH = os.path.dirname(os.path.realpath(__file__))

def play_signal_sound():
	question_start_signal = os.path.join(os.path.dirname(CURRENT_FOLDER_PATH), "sounds/question_start_signal.wav")
	chunk = 1024
	# 打开 .wav 音频文件
	f = wave.open(question_start_signal, 'rb')
	# 初始化pyaudio
	p = pyaudio.PyAudio()
	# 打开一个stream
	stream = p.open(
	format = p.get_format_from_width(f.getsampwidth()),
	channels = f.getnchannels(),
	rate = f.getframerate(),
	output = True
	)
	# 读取音频文件中的数据
	data = f.readframes(chunk)

	# 播放音频文件
	while data != '':
	stream.write(data)
	data = f.readframes(chunk)

	# 终止stream
	stream.stop_stream()
	stream.close()
	# 关闭pyaudio
	p.terminate()

def get_audio():
  recorder = pyaudio.PyAudio()