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
import wave
import wave
import rospy
import pyaudio
from std_msgs.msg import Int8
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient


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

class RobotSpeech(object):
	def __init__(self):
		"""
			初始化函数，初始化录音以及语音识别api需要用到的各项参数
		"""
		# 初始化ros node
		rospy.init_node("robot_speech")

		# Soundplay 参数
		self.voice = rospy.get_param("~voice", "voice_kal_diphone")
		self.speaker = SoundClient(blocking=True)
		rospy.sleep(1)
		self.speaker.stopAll()
		rospy.sleep(1)

		# 录音参数
		self.start_record = False
		self.stop_record = False
		self.count = 0
		self.chunk = 256
		self.format = pyaudio.paInt16
		self.channels = 1
		self.rate = 11025
		self.record_second = 11

		# api参数
		self.URL = "http://api.xfyun.cn/v1/service/v1/iat"
		self.APPID = "15944331"
		self.API_KEY = "bpNdxEhagyalZVtC6fddFeGZ"
		self.SECRET_KEY = '1QRtmDnXAA9TUQuGH8zHL5K1OW2GnpDu'
		self.audio_folder = os.path.join(os.path.dirname(CURRENT_FOLDER_PATH),"/sounds/gpsr_record/")
		self.ASR_URL = 'http://vop.baidu.com/server_api'
		self.DEV_PID = 1737
		self.CUID = '123456PYTHON'
		self.FORMAT = 'wav'
		self.RATE = 16000

	def setup_recorder(self):
		self.recorder = pyaudio.PyAudio()

	def speaker_say(self, sentence):
		self.speaker.say(sentence, self.voice)
	
	def keyword_callback(self, msg):
		if msg.data.lower().strip() == "jack":
			self.get_audio()
		
		if msg.data.lower().strip() == "ok" or msg.data.lower().strip() == "okay":
			self.stop_record = True
	
	def control_callback(self, msg):
		if msg.action == "speak":
			if msg.attributes.speech.sentence:
				# Speak target sentence
				self.speaker_say(msg.attributes.speech.sentence)
		if msg.action == "recognition" and msg.target == "speech":
			# speech recognition start
			self.speaker_say("Now you can speak to me")
			self.speaker_say("Please speak to me after hearing")
			play_signal_sound()

	def get_audio(self):
		self.setup_recorder()
		

test = RobotSpeech()
test.speaker_say("Now you can speak to me")
test.speaker_say("Please speak to me after hear ring")
play_signal_sound()