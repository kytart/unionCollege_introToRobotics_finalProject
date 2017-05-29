# Union College: CSC-325-01 Intro to Robotics Class - Final Project

A ROS package that makes a turtlebot act like a dog, taking voice commands and visually recognising objects to fetch.


## Requirements

* [ROS Indigo or later](http://wiki.ros.org/) 
* Python 2.6, 2.7 or 3.3+
* Microphone
* Audio speakers


## Install

1. Check if you have to install any system packages, required by [PyAudio](https://people.csail.mit.edu/hubert/pyaudio/#downloads).
2. Install required python packages:

```
sudo pip install -r requirements.txt
```

3. Create file `google_api_credentials.json` in the project root directory, that contains your [Google Cloud](https://cloud.google.com/) API credentials in JSON format. 
You must enable [Google Cloud Speech](https://cloud.google.com/speech/) API in the Google console.


## Launch

Convenient launch file is available. Run command:

```
roslaunch dogbot dogbot.launch
```

## Controls

Dogbot is controlled entirely with voice.

### Available Voice Commands

1. "Come here!" - Dogbot is excited and moves 1 meter ahead.
2. "Go away!" - Dogbot is sad, turns around and moves 1 meter ahead.
3. "Come back!" - Dogbot is happy, turns around and moves 1 meter ahead.
4. "Good boy!" - Dogbot is happy.
5. "Turn around!" - Dogbot is excited and turns around by 360 degrees.
6. "Shake it!" or "Shake that booty!" - Bonus secret command.

### Emotions

Dogbot is not just a robot. It has emotions. To allow Dogbot express emotions, make sure the system audio volume is turned up.
