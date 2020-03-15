#!/usr/bin/env python

import math
import numpy as np

import BNO055

import rospy
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from bno055.msg import bno055_info
from std_srvs.srv import Trigger, TriggerResponse
import sys, os

def save_calibration(req):
	global sensor
	response = TriggerResponse()
	try:
		calibration = np.array(sensor.get_calibration())
		np.save(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'calibration'), calibration)
		response.success = True
		response.message = str(calibration)
	except Exception as e:
		response.success = False
		response.message = str(e)
		return response
	return response

def publisher():

	rospy.init_node('imu')

	dataPub = rospy.Publisher('/imu/data', Imu, queue_size=3)
	infoPub = rospy.Publisher('/imu/info', bno055_info, queue_size=3)
	headingPub = rospy.Publisher('/imu/heading', Float64, queue_size=3)

	load_calibration = rospy.get_param("~load_calibration", False)

	rate = rospy.Rate(30) #30Hz data read

	# Setup BNO055
	# Create and configure the BNO sensor connection.
	# Raspberry Pi configuration with I2C and RST connected to GPIO 27:
	global sensor
	sensor = BNO055.BNO055()

	try:
		sensor.begin()
	except Exception as e:
		rospy.logerr('Failed to initialize BNO055! %s', e)
		sys.exit(1)

	#sensor.set_axis_remap(BNO055.AXIS_REMAP_Y, BNO055.AXIS_REMAP_X, BNO055.AXIS_REMAP_Z) #swap x,y axis

	# Print system status and self test result.
	try:
		status, self_test, error = sensor.get_system_status()
	except Exception as e:
		rospy.logerr('Failed to read BNO055 system status! %s', e)
		sys.exit(2)

	rospy.logdebug('System status: %s', status)
	rospy.logdebug('Self test result (0x0F is normal): %s', hex(self_test))
	# Print out an error if system status is in error mode.
	if status == 0x01:
		rospy.logerr('System error: %s', error)
		rospy.logerr('See datasheet section 4.3.59 for the meaning.')
		sys.exit(3)

	# Print BNO055 software revision and other diagnostic data.
	try:
		sw, bl, accel, mag, gyro = sensor.get_revision()
	except Exception as e:
		rospy.logerr('Failed to read BNO055 meta-inforamtion! %s', e)
		sys.exit(4)

	rospy.loginfo('Software version:   %d', sw)
	rospy.loginfo('Bootloader version: %d', bl)
	rospy.loginfo('Accelerometer ID:   %s', hex(accel))
	rospy.loginfo('Magnetometer ID:    %s', hex(mag))
	rospy.loginfo('Gyroscope ID:       %s', hex(gyro))

	rospy.loginfo('Reading BNO055 data...')

	if load_calibration:
		try:
			sensor.set_calibration(np.load(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'calibration.npy')).tolist())
		except Exception as e:
			rospy.logerr("Error loading calibration data: " + str(e))

	save_calibration_srv = rospy.Service('/imu/save_calibration', Trigger, save_calibration)

	while not rospy.is_shutdown():
		# Define messages
		msg = Imu()
		info = bno055_info()
		heading = Float64()

		orientation = Quaternion()
		angular_vel = Vector3()
		linear_accel =  Vector3()

		# Update meta data
		attempts = 0
		sys_cal = 0
		temp_c = 0
		while attempts < 4:
			try:
				# Read the calibration status, 0=uncalibrated and 3=fully calibrated.
				sys_cal, gyro, accel, mag = sensor.get_calibration_status()
				temp_c = sensor.read_temp()
				break
			except Exception as e:
				rospy.logerr('Failed to read BNO055 calibration stat and temp! %s', e)
				attempts += 1
				rospy.sleep(0.01)

		if attempts != 4:
			info.sysCalibration = sys_cal
			info.accelCalibration = accel
			info.gyroCalibration = gyro
			info.magnoCalibration = mag
			info.tempC = temp_c

		# Update real data
		attempts = 0
		while attempts < 4:
			try:
				# Orientation as a quaternion:
				orientation.x, orientation.y, orientation.z, orientation.w  = sensor.read_quaternion()

				# Gyroscope data (in degrees per second converted to radians per second):
				gry_x, gry_y, gry_z = sensor.read_gyroscope()
				angular_vel.x = math.radians(gry_x)
				angular_vel.y = math.radians(gry_y)
				angular_vel.z = math.radians(gry_z)

				# Linear acceleration data (i.e. acceleration from movement, not gravity--
    			# returned in meters per second squared):
				linear_accel.x, linear_accel.y, linear_accel.z = sensor.read_linear_acceleration()
				heading.data = sensor.read_euler()[0]
				break
			except Exception as e:
				rospy.logerr('Failed to read BNO055 data! %s', e)
				attempts += 1
				rospy.sleep(0.01)

		if attempts != 4:
			msg.orientation = orientation
			msg.orientation_covariance[0] = 0.01 # covariance in x,y,z (3x3)
			msg.orientation_covariance[4] = 0.01
			msg.orientation_covariance[8] = 0.01

			msg.angular_velocity = angular_vel
			msg.angular_velocity_covariance[0] = 0.01 # covariance in x,y,z (3x3)
			msg.angular_velocity_covariance[4] = 0.01
			msg.angular_velocity_covariance[8] = 0.01

			msg.linear_acceleration = linear_accel
			msg.linear_acceleration_covariance[0] = 0.01 # covariance in x,y,z (3x3)
			msg.linear_acceleration_covariance[4] = 0.01
			msg.linear_acceleration_covariance[8] = 0.01


		#update message headers
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = 'imu_link'

		dataPub.publish(msg)

		info.header.stamp = rospy.Time.now()
		info.header.frame_id = 'imu_link'

		infoPub.publish(info)

		headingPub.publish(heading)

		rate.sleep()

if __name__ == '__main__':
	try:
		publisher()
	except rospy.ROSInterruptException:
		pass

# Unused functions
# Read the Euler angles for heading, roll, pitch (all in degrees).
# heading, roll, pitch = sensor.read_euler()
# Magnetometer data (in micro-Teslas):
# x,y,z = sensor.read_magnetometer()
# Accelerometer data (in meters per second squared):
# x,y,z = sensor.read_accelerometer()
# Gravity acceleration data (i.e. acceleration just from gravity--returned
# in meters per second squared):
# x,y,z = sensor.read_gravity()
