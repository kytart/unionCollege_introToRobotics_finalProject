#!/usr/bin/env python
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int16
from emotions import HAPPY, SAD, EXCITED, MUSIC

COMMAND_TURN_AROUND = 'turn around'
COMMAND_GOOD_BOY = 'good boy'
COMMAND_COME_HERE = 'come here'
COMMAND_GO_AWAY = 'go away'
COMMAND_COME_BACK = 'come back'
COMMAND_SHAKE_IT = 'shake it'
COMMAND_SHAKE_THAT_BOOTY = 'shake that booty'
COMMAND_FETCH = 'fetch'
COMMAND_SEARCH = 'search'

MAX_LINEAR_SPEED = 0.2

pubMove = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
pubEmotion = rospy.Publisher('/dogbot/emotion', String, queue_size=10)


class Vision:
	def __init__(self):
		self.x = None
		self.radius = None

	def set_x(self, x):
		self.x = x.data

	def set_radius(self, radius):
		self.radius = radius.data if radius.data >= 0 else None


def calculate_distance(x1, x0, y1, y0):
	return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)


def move_and_rotate(distance=0.0, is_forward=True, relative_angle=0.0, is_clockwise=True, linear_speed=0.2, angular_speed=1.0):
	relative_angle = math.degrees(relative_angle)

	current_distance = None
	current_angle = None
	rate = rospy.Rate(10)
	listener = tf.TransformListener()

	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
			rot_radians = tf.transformations.euler_from_quaternion(rot)[2]

			if current_distance is None:
				current_distance = 0
				current_angle = 0
				start_trans = trans
				start_rot = rot_radians
			else:
				current_distance = calculate_distance(trans[0], start_trans[0], trans[1], start_trans[1])
				current_angle = math.degrees(abs(rot_radians - start_rot)) % 360
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		reached_distance = current_distance >= distance or linear_speed <= 0
		reached_rotation = current_angle >= relative_angle or angular_speed <= 0

		if reached_distance and reached_rotation:
			break

		out_data = Twist()

		if reached_distance:
			out_data.linear.x = 0.0
		else:
			linear_speed = min(MAX_LINEAR_SPEED, linear_speed)
			out_data.linear.x = linear_speed if is_forward else -linear_speed

		if reached_rotation:
			out_data.angular.z = 0.0
		else:
			out_data.angular.z = -angular_speed if is_clockwise else angular_speed

		pubMove.publish(out_data)
		rate.sleep()

	out_data = Twist()
	out_data.linear.x = 0.0
	out_data.angular.z = 0.0
	pubMove.publish(out_data)
	rate.sleep()


def move(distance, is_forward):
	move_and_rotate(distance=distance, is_forward=is_forward)


def rotate(relative_angle, is_clockwise, speed=1.0):
	move_and_rotate(relative_angle=relative_angle, is_clockwise=is_clockwise, angular_speed=speed)


def dance(radius):
	speed = 2.5
	rotate(radius, True, speed)
	rotate(radius * 2, False, speed)
	rotate(radius * 2, True, speed)
	rotate(radius, False, speed)


def turn_around():
	rotate(math.radians(180.0), True)
	rotate(math.radians(180.0), True)


def turn_and_go():
	rotate(math.radians(90.0), True)
	move(1.0, False)


def follow_ball(vision):
	radius_per_meter = 25
	distance_threshold = 0.1
	distance_multiplier = 0.4

	rotation_step = 10
	rotation_degrees_per_step = 2.0
	rotation_degrees_threshold = 2.0

	max_distance = 1.0
	max_angle = math.radians(15.0)

	rate = rospy.Rate(1)

	while not rospy.is_shutdown() and vision.x is not None and vision.radius is not None:
		meters_distance = radius_per_meter / vision.radius
		degrees_distance = (abs(vision.x) / rotation_step) * rotation_degrees_per_step
		radians_distance = math.radians(degrees_distance)
		is_to_the_left = vision.x < 0

		if meters_distance > distance_threshold:
			linear_speed = meters_distance
		else:
			linear_speed = 0.0

		if degrees_distance > rotation_degrees_threshold:
			angular_speed = math.radians(10.0)
		else:
			angular_speed = 0.0

		if linear_speed == 0.0 and angular_speed == 0.0:
			break

		move_and_rotate(
			distance=min(meters_distance * distance_multiplier, max_distance),
			is_forward=True,
			relative_angle=min(abs(radians_distance), max_angle),
			is_clockwise=not is_to_the_left,
			linear_speed=linear_speed,
			angular_speed=angular_speed
		)

		rate.sleep()

	return vision.x is not None and vision.radius is not None


def perform_command(command, vision):
	if command.data == COMMAND_TURN_AROUND:
		pubEmotion.publish(EXCITED)
		rotate(math.radians(180.0), True)
		rotate(math.radians(180.0), True)
	elif command.data == COMMAND_GOOD_BOY:
		pubEmotion.publish(HAPPY)
		dance(math.radians(5.0))
	elif command.data == COMMAND_COME_HERE:
		pubEmotion.publish(EXCITED)
		move(1.0, True)
	elif command.data == COMMAND_GO_AWAY:
		pubEmotion.publish(SAD)
		rotate(math.radians(180.0), True)
		move(1.0, True)
	elif command.data == COMMAND_COME_BACK:
		pubEmotion.publish(HAPPY)
		rotate(math.radians(180.0), True)
		move(1.0, True)
	elif command.data == COMMAND_SHAKE_IT or command.data == COMMAND_SHAKE_THAT_BOOTY:
		pubEmotion.publish(MUSIC)
		dance(math.radians(5.0))
		dance(math.radians(5.0))
	elif command.data == COMMAND_FETCH or command.data == COMMAND_SEARCH:
		pubEmotion.publish(HAPPY)
		success = False
		for x in range(0, 5):
			success = follow_ball(vision)
			if success:
				break

		if success:
			pubEmotion.publish(HAPPY)
		else:
			pubEmotion.publish(SAD)

	else:
		rospy.logerr('Unrecognized command "%s"', command.data)


def init():
	vision = Vision()

	rospy.init_node('dogbot', anonymous=True)
	rospy.Subscriber("/dogbot/voice_command", String, perform_command, vision)
	rospy.Subscriber("/x_ball", Int16, vision.set_x)
	rospy.Subscriber("/ball_radius", Int16, vision.set_radius)

	rospy.spin()


if __name__ == '__main__':
	init()
