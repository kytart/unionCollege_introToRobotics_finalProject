# Union College: CSC-325-01 Intro to Robotics Class - Final Project

A ROS package that makes a turtlebot act like a dog, taking voice commands and visually recognizing objects to fetch.


## Requirements

* [ROS Indigo or later](http://wiki.ros.org/)
* [Turtlebot](http://wiki.ros.org/Robots/TurtleBot) or similar moving base
* Python 2.6, 2.7 or 3.3+
* Microphone
* Audio speakers
* Camera


## Install

1. Check if you have to install any system packages, required by [PyAudio](https://people.csail.mit.edu/hubert/pyaudio/#downloads).
2. Install required python packages:

```
sudo pip install -r requirements.txt
```

3. Create file `google_api_credentials.json` in the project root directory, that contains your [Google Cloud](https://cloud.google.com/) API credentials in JSON format. 
You must enable [Google Cloud Speech](https://cloud.google.com/speech/) API in the Google console.

If you have any issues and instructions provided here don't help, try following troubleshooting instructions on [this page](https://pypi.python.org/pypi/SpeechRecognition/). 

## Launch

### With Turtlebot

Run launch file:

```
roslaunch dogbot with_turtlebot.launch
```

### With different robot 

1. Launch moving base and camera.
2. Launch Dogbot:

```
roslaunch dogbot basic.launch
```

## Controls

Dogbot is controlled entirely with voice.

### Available Voice Commands

1. "Come here!" - Dogbot is excited and moves 1 meter ahead.
2. "Go away!" - Dogbot is sad, turns around and moves 1 meter ahead.
3. "Come back!" - Dogbot is happy, turns around and moves 1 meter ahead.
4. "Good boy!" - Dogbot is happy.
5. "Turn around!" - Dogbot is excited and turns around by 360 degrees.
7. "Fetch!" or "Search!" - Dogbot is happy and starts chasing the ball.
6. "Shake it!" or "Shake that booty!" - Bonus secret command.


## Fetching the ball


## Emotions

Dogbot is not just a robot. It has emotions. To allow Dogbot express emotions, make sure the system audio volume is turned up.


## Troubleshooting

### Microphone sensitivity

Depending on what microphone you're using and how it's set in the system, you might need to specify microphone sensitivity.
When using launch file, `mic_sensitivity` argument is available. Default value is 3000.

```
roslaunch dogbot with_turtlebot.launch mic_sensitivity:=3000
# or
roslaunch dogbot basic.launch mic_sensitivity:=3000
```

There are 3 possible scenarios:

* Voice commands are recognized with default microphone sensitivity.
    * No need to do anything.
* Speech handler is trying to recognize words even when you're not speaking.
    * Use higher value for microphone sensitivity.
* Speech handler doesn't recognize when you're speaking.
    * Use lower value for microphone sensitivity.
    
To debug speech handler, inspect logs. Recognized phrases are logged as debug logs and any errors are logged as errors.
