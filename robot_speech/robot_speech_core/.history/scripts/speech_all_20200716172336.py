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
import base64
import pyaudio
from std_msgs.msg import Int8
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient

IS_PY3 = sys.version_info.major == 3

if IS_PY3:
	from urllib.request import urlopen
	from urllib.request import Request
	from urllib.error import URLError
	from urllib.parse import urlencode
	timer = time.perf_counter
else:
	from urllib2 import urlopen
	from urllib2 import Request
	from urllib2 import URLError
	from urllib import urlencode
	if sys.platform == "win32":
			timer = time.clock
	else:
			# On most other platforms the best timer is time.time()
			timer = time.time
class DemoError(Exception):
    pass


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
		self.record_format = pyaudio.paInt16
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
		self.AUDIO_FORMAT = 'wav'
		self.RATE = 16000

	def fetch_token(self):
		params = {'grant_type': 'client_credentials',
						'client_id': self.API_KEY,
						'client_secret': self.SECRET_KEY}
		post_data = urlencode(params)
		if (IS_PY3):
			post_data = post_data.encode( 'utf-8')
		req = Request(TOKEN_URL, post_data)
		try:
			f = urlopen(req)
			result_str = f.read()
		except URLError as err:
			result_str = err.read()
		if (IS_PY3):
			result_str =  result_str.decode()

		result = json.loads(result_str)
		if ('access_token' in result.keys() and 'scope' in result.keys()):
			if not SCOPE in result['scope'].split(' '):
				raise DemoError('scope is not correct')
			print('SUCCESS WITH TOKEN: %s ; EXPIRES IN SECONDS: %s' % (result['access_token'], result['expires_in']))
			return result['access_token']
		else:
			raise DemoError('MAYBE API_KEY or SECRET_KEY not correct: access_token or scope not found in token response')

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
			self.speaker_say("Please say jack to wake me up")

	def get_audio(self):
		self.setup_recorder()
		play_signal_sound()
		file_name = self.audio_folder + "_" + str(self.count) + ".wav"
		print ("[INFO] Start to record input audio and save to file: %s"%(file_name))
		stream = self.recorder.open(
			format = self.record_format,
			channels = self.channels,
			rate = self.rate,
			input=True,
			frames_per_buffer=self.chunk
		)
		frames = []
		for i in range(self.rate/self.chunk*self.record_second):
			if self.stop_record:
				print ("[INFO] Stop recording.")
				break
			data = stream.read(self.chunk)
			frames.append(data)
		print ("[INFO] Recording finished. Saving record to {}..".format(file_name))

		stream.stop_stream()
		stream.close()
		self.recorder.terminate()

		wf = wave.open(file_name, 'wb')
		wf.setnchannels(self.channels)
		wf.writeframes(b''.join(frames))
		wf.close()

		self.count +=1 
	
	def speech_recognition(self):
		token = self.fetch_token()
		audio_index = str(self.count)
		path_to_wav = self.audio_folder + "gpsr_" + audio_index + ".wav"
		speech_data = []
		with open(path_to_wav, 'rb') as speech_file:
				speech_data = speech_file.read()
		length = len(speech_data)
		speech = base64.b64encode(speech_data)
		if (IS_PY3):
				speech = str(speech, 'utf-8')
		params = {'dev_pid': self.DEV_PID,
						'format': self.AUDIO_FORMAT,
						'rate': self.RATE,
						'token': token,
						'cuid': self.CUID,
						'channel': 1,
						'speech': speech,
						'len': length
						}  




test = RobotSpeech()
test.speaker_say("Now you can speak to me")
test.speaker_say("Please speak to me after hear ring")
play_signal_sound()