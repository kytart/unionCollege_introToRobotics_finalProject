#!/usr/bin/env python
import math
import rospy
import tf
from geometry_msgs.msg import Twist

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
	speed = 0.2
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
				current_angle = abs(rot_radians - start)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		print 'current angle: {}'.format(math.degrees(current_angle))

		if current_angle >= relative_angle:
			break

		pub.publish(out_data)
		rate.sleep()

	out_data.angular.z = 0.0
	pub.publish(out_data)
	rate.sleep()


def make_square(size):
	move(size, True)
	rotate(math.radians(90.0), True)
	move(size, True)
	rotate(math.radians(90.0), True)
	move(size, True)
	rotate(math.radians(90.0), True)
	move(size, True)


def init():
	rospy.init_node('dogbot', anonymous=True)

if __name__ == '__main__':
	init()
