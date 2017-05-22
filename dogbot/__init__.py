import os

from speech_handler import SpeechHandler

GOOGLE_API_CREDENTIALS_JSON = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../google_api_credentials.json')

with open(GOOGLE_API_CREDENTIALS_JSON, 'r') as credentialsFile:
	googleApiCredentials = credentialsFile.read()

speechHandler = SpeechHandler(googleApiCredentials)
