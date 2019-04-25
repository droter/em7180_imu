#!/usr/bin/env python

'''
	publisherEM7180.py uses parts of
	mastertest.py: Example Python script for running EM7180 SENtral sensor hub in master mode.

	Copyright (C) 2018 Simon D. Levy

	This file is part of EM7180.

	EM7180 is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	EM7180 is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with EM7180.  If not, see <http://www.gnu.org/licenses/>.
'''

from em7180.em7180_utils import EM7180_Master
import rospy
import math
import time
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import MagneticField, Imu, Temperature, FluidPressure
from std_msgs.msg import Float64


MAG_RATE       = 100  # Hz
ACCEL_RATE     = 200  # Hz
GYRO_RATE      = 200  # Hz
BARO_RATE      = 50   # Hz
Q_RATE_DIVISOR = 3    # 1/3 gyro rate

seq = 0

# Initilize IMU
em7180 = EM7180_Master(MAG_RATE, ACCEL_RATE, GYRO_RATE, BARO_RATE, Q_RATE_DIVISOR)

# Start the EM7180 in master mode
if not em7180.begin():
	print(em7180.getErrorString())
	exit(1)

em7180.checkEventStatus()

if em7180.gotError():
	print('ERROR: ' + em7180.getErrorString())
	exit(1)


# Initialize node
rospy.init_node('EM7180_imu_driver', anonymous=False)

# Publisher
mag_pub=rospy.Publisher('imu/mag',MagneticField,queue_size=10)
temp_pub=rospy.Publisher('sensor/temp', Temperature ,queue_size=10)
pressure_pub=rospy.Publisher('sensor/pressure', FluidPressure ,queue_size=10)
alt_pub=rospy.Publisher('sensor/alt', Float64 ,queue_size=10)
imu_pub=rospy.Publisher('imu/data', Imu , queue_size=10)
imu_raw_pub=rospy.Publisher('imu/data_raw', Imu , queue_size=10)

imu_yaw_calibration = rospy.get_param('~imu_yaw_calibration', 0.0)
declination = rospy.get_param('~declination', 0.0)


while not rospy.is_shutdown():

	rate=rospy.Rate(50)

	if (em7180.gotQuaternion()):

		qw, qx, qy, qz = em7180.readQuaternion()

		# IMU data http://www.ros.org/reps/rep-0145.html
		# Coordinate Conventions http://www.ros.org/reps/rep-0103.html
		# This data is prepared to be fused with Robot_Localization
		# Yaw is positive when rotated counter clockwise.
		# Pitch is positive when nose is down.
		# Roll is positive when left side is up.
		# Yaw is zero when pointing East
		
		roll  = math.atan2(2.0 * (qw * qx + qy * qz), qw * qw - qx * qx - qy * qy + qz * qz)
		pitch = math.asin(2.0 * (qx * qz - qw * qy))
		yaw   = -math.atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz)   

		# change from radian to degrees
		pitch *= 180.0 / math.pi
		yaw   *= 180.0 / math.pi

		# change yaw so East is zero degrees
		yaw +=90

		# get declination and yaw calibration offset in degrees
		# These are set in paramater server
		yaw   += declination # Lookup: http://www.magnetic-declination.com/
		yaw   += imu_yaw_calibration
		if yaw < 0: yaw   += 360.0  # Ensure yaw stays between 0 and 360

		roll  *= 180.0 / math.pi

		#print('Quaternion Roll, Pitch, Yaw: %+2.2f %+2.2f %+2.2f' % (roll, pitch, yaw))

		# change from degrees to radians
		roll *= math.pi / 180.0
		pitch *= math.pi / 180.0
		yaw *= math.pi / 180.0

		q = quaternion_from_euler(roll,pitch,yaw)

		# Set IMU variable
		imuMsg = Imu()

		imuMsg.orientation.x = q[0]
		imuMsg.orientation.y = q[1]
		imuMsg.orientation.z = q[2]
		imuMsg.orientation.w = q[3]

		imuMsg.orientation_covariance = [
		0.0025 , 0 , 0,
		0, 0.0025, 0,
		0, 0, 0.0025
		]

		imuMsg.header.stamp= rospy.Time.now()
		imuMsg.header.frame_id = 'imu_link'
		imuMsg.header.seq = seq
		seq = seq + 1

		# IMU raw no orintation
		imuRawMsg = Imu()

		imuRawMsg.orientation.x = 0
		imuRawMsg.orientation.y = 0
		imuRawMsg.orientation.z = 0
		imuRawMsg.orientation.w = 0

		imuRawMsg.orientation_covariance = [
		0, 0, 0,
		0, 0, 0,
		0, 0, 0
		]

		imuRawMsg.header.stamp= rospy.Time.now()
		imuRawMsg.header.frame_id = 'imu_link'


	if em7180.gotAccelerometer():

		ax,ay,az = em7180.readAccelerometer()
		
		#print('Accel: %+3.3f %+3.3f %+3.3f' % (ax,ay,az))

		# IMU data
		imuMsg.linear_acceleration.x=ax
		imuMsg.linear_acceleration.y=ay
		imuMsg.linear_acceleration.z=az

		imuMsg.linear_acceleration_covariance = [
		0.04 , 0 , 0,
		0 , 0.04, 0,
		0 , 0 , 0.04
		]

		# IMU raw data
		imuRawMsg.linear_acceleration.x=ax
		imuRawMsg.linear_acceleration.y=ay
		imuRawMsg.linear_acceleration.z=az

		imuRawMsg.linear_acceleration_covariance = [
		0.04 , 0 , 0,
		0 , 0.04, 0,
		0 , 0 , 0.04
		]


	if em7180.gotGyrometer():

		gx,gy,gz = em7180.readGyrometer()

		#print('Gyro: %+3.3f %+3.3f %+3.3f' % (gx,gy,gz))

		#  Or define output variable according to the Android system, where
		#  heading (0 to 360) is defined by the angle between the y-axis and True
		#  North, pitch is rotation about the x-axis (-180 to +180), and roll is
		#  rotation about the y-axis (-90 to +90) In this systen, the z-axis is
		#  pointing away from Earth, the +y-axis is at the 'top' of the device
		#  (cellphone) and the +x-axis points toward the right of the device.

		# IMU data
		imuMsg.angular_velocity.x=gx
		imuMsg.angular_velocity.y=gy
		imuMsg.angular_velocity.z=gz

		imuMsg.angular_velocity_covariance = [
		0.02, 0 , 0,
		0 , 0.02, 0,
		0 , 0 , 0.02
		]

		# IMU raw data 
		imuRawMsg.angular_velocity.x=gx
		imuRawMsg.angular_velocity.y=gy
		imuRawMsg.angular_velocity.z=gz

		imuRawMsg.angular_velocity_covariance = [
		0.02, 0 , 0,
		0 , 0.02, 0,
		0 , 0 , 0.02
		]

		# publish message
		imu_pub.publish(imuMsg)
		imu_raw_pub.publish(imuRawMsg)


	if em7180.gotBarometer():

		pressure, temperature = em7180.readBarometer()

		altitude = (1.0 - math.pow(pressure / 1013.25, 0.190295)) * 44330
		#print('  Altitude = %2.2f m\n' % altitude) 
	
		# Set Pressure variables
		pressMsg = FluidPressure()
		pressMsg.header.stamp = rospy.Time.now()
		pressMsg.header.frame_id="imu_link"
		pressMsg.fluid_pressure = pressure
		pressMsg.variance = 0

		# Set Temperature variables
		tempMsg = Temperature()
		tempMsg.header.stamp = rospy.Time.now()
		tempMsg.header.frame_id="imu_link"
		tempMsg.temperature = temperature
		tempMsg.variance = 0

		# Set Altitude variables
		altMsg = altitude

		# publish message
		temp_pub.publish(tempMsg)
		pressure_pub.publish(pressMsg)
		alt_pub.publish(altMsg)


	if em7180.gotMagnetometer():

		mx,my,mz = em7180.readMagnetometer()	

		# Magnetic field vector
		magneticVector = MagneticField()
		magneticVector.header.stamp=rospy.Time.now()
		magneticVector.header.frame_id="imu_link"
		magneticVector.magnetic_field.x=mx
		magneticVector.magnetic_field.y=my
		magneticVector.magnetic_field.z=mz
		magneticVector.magnetic_field_covariance=[2,0,0,0,2,0,0,0,4]

		# publish message
		mag_pub.publish(magneticVector)


	# Info to ros_console and screen
	#rospy.loginfo("Publishing sensor data from IMU")

	rate.sleep()



