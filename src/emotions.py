#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import String
import pyaudio
import wave

HAPPY = 'happy'
SAD = 'sad'
EXCITED = 'excited'
MUSIC = 'music'

CHUNK = 1024

PATH_TO_AUDIO_FILES = os.path.join(os.path.dirname(os.path.realpath(__file__)) + '/../media/audio/')

def play(file):
	p = pyaudio.PyAudio()
	wf = wave.open(PATH_TO_AUDIO_FILES + file, 'rb')
	stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
					channels=wf.getnchannels(),
					rate=wf.getframerate(),
					output=True)
	data = wf.readframes(CHUNK)

	while data != '':
		stream.write(data)
		data = wf.readframes(CHUNK)

	stream.stop_stream()
	stream.close()
	p.terminate()


def express_emotion(emotion):
	if emotion.data == HAPPY:
		play('happy_dog.wav')
	elif emotion.data == SAD:
		play('sad_dog.wav')
	elif emotion.data == EXCITED:
		play('excited_dog.wav')
	elif emotion.data == MUSIC:
		play('dance_dog.wav')
	else:
		print 'unrecognized emotion "{}"'.format(emotion.data)


def init():
	rospy.init_node('dogbotEmotions', anonymous=True)
	rospy.Subscriber("/dogbot/emotion", String, express_emotion)
	rospy.spin()


if __name__ == '__main__':
	init()
