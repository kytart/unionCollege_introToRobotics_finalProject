#!/usr/bin/env python
import speech_recognition as sr


class SpeechHandler:
	def __init__(self, google_api_credentials_json):
		self._google_api_credentials = google_api_credentials_json
		self._recognizer = sr.Recognizer()

	def __enter__(self):
		return self

	def listen(self):
		with sr.Microphone() as source:
			audio = self._recognizer.listen(source)
		return self._recognizer.recognize_google_cloud(audio, credentials_json=self._google_api_credentials)
