#!/usr/bin/env python
import os
import rospy
import speech_recognition as sr
from std_msgs.msg import String

GOOGLE_API_CREDENTIALS_JSON_PATH = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../google_api_credentials.json')


def init():
	rospy.init_node('dogbotVoiceRecognizer', anonymous=True)
	pub = rospy.Publisher('/dogbot/voice_command', String, queue_size=10)
	rate = rospy.Rate(1)
	recognizer = sr.Recognizer()

	with open(GOOGLE_API_CREDENTIALS_JSON_PATH, 'r') as credentialsFile:
		google_api_credentials = credentialsFile.read()

	with sr.Microphone() as source:
		while not rospy.is_shutdown():
			audio = recognizer.listen(source)
			try:
				phrase = recognizer.recognize_google_cloud(audio, credentials_json=google_api_credentials)
				pub.publish(phrase)
			except sr.UnknownValueError:
				rospy.logerr('Google Cloud Speech could not understand audio')
			except sr.RequestError as e:
				rospy.logerr('Could not request results from Google Cloud Speech service; {0}'.format(e))
			rate.sleep()


if __name__ == '__main__':
	init()
