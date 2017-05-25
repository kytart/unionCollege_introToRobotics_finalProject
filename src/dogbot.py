#!/usr/bin/env python
import math
import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import String

COMMAND_TURN_AROUND = 'turn around'
COMMAND_GOOD_BOY = 'good boy'
COMMAND_COME_HERE = 'come here'
COMMAND_GO_AWAY = 'go away'
COMMAND_COME_BACK = 'come back'
COMMAND_SHAKE_IT = 'shake it'
COMMAND_SHAKE_THAT_BOOTY = 'shake that booty'

pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)


def calculate_distance(x1, x0, y1, y0):
	return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)


def move(distance, is_forward):
	speed = 0.2
	current_distance = None
	rate = rospy.Rate(10)
	listener = tf.TransformListener()

	out_data = Twist()
	out_data.linear.x = speed if is_forward else -speed

	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
			if current_distance is None:
				current_distance = 0
				start = trans

			else:
				current_distance = calculate_distance(round(trans[0]), start[0], trans[1], start[1])
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		print 'start: {}'.format(start)
		print 'current: {}'.format(trans)
		print 'distance: {}'.format(current_distance)
		print '=========================='

		if current_distance >= distance:
			break

		pub.publish(out_data)
		rate.sleep()

	out_data.linear.x = 0.0
	pub.publish(out_data)
	rate.sleep()


def rotate(relative_angle, is_clockwise):
	speed = 2.0
	current_angle = None
	rate = rospy.Rate(10)
	listener = tf.TransformListener()

	out_data = Twist()
	out_data.angular.z = -speed if is_clockwise else speed

	while not rospy.is_shutdown():
		try:
			(trans, rot) = listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
			rot_radians = tf.transformations.euler_from_quaternion(rot)[2]

			if current_angle is None:
				current_angle = 0
				start = rot_radians
			else:
				current_angle = math.degrees(abs(rot_radians - start)) % 360
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		print 'start: {}'.format(math.degrees(start))
		print 'current: {}'.format(math.degrees(rot_radians))
		print 'current angle: {}'.format(current_angle)
		print '=========================='

		if current_angle >= relative_angle:
			break

		pub.publish(out_data)
		rate.sleep()

	out_data.angular.z = 0.0
	pub.publish(out_data)
	rate.sleep()


def dance(radius):
	rotate(radius, True)
	rotate(radius * 2, False)
	rotate(radius * 2, True)
	rotate(radius * 2, False)
	rotate(radius, True)


def turn_around():
	rotate(180.0, True)
	rotate(180.0, True)


def turn_and_go():
	rotate(90.0, True)
	move(1.0, False)


def perform_command(command):
	print 'command "{}"'.format(command.data)

	if command.data == COMMAND_TURN_AROUND:
		rotate(180.0, True)
		rotate(180.0, True)
	elif command.data == COMMAND_GOOD_BOY:
		dance(5.0)
	elif command.data == COMMAND_COME_HERE:
		move(1.0, True)
	elif command.data == COMMAND_GO_AWAY or command.data == COMMAND_COME_BACK:
		rotate(180.0, True)
		move(1.0, True)
	elif command.data == COMMAND_SHAKE_IT or command.data == COMMAND_SHAKE_THAT_BOOTY:
		dance(5.0)
		dance(5.0)
	else:
		print 'Unrecognized command "{}"'.format(command.data)


def init():
	rospy.init_node('dogbot', anonymous=True)
	rospy.Subscriber("/dogbot/voice_command", String, perform_command)
	rospy.spin()


if __name__ == '__main__':
	init()
