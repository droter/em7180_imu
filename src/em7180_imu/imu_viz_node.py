#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) Vortex NTNU.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## imu_viz_node.py subscribes to ROS topics with IMU data and displays

import rospy
import math
import time
from datetime import datetime
from std_msgs.msg import String
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import MagneticField, Imu, Temperature, FluidPressure
from Tkinter import *

# GUI stuff
root = Tk()
root.geometry("900x700")

# Global variables to handle all the data fetched from the sensor
gtemp = StringVar(root) # Temperature
gpitch = DoubleVar(root) # Pitch in Degrees (y)
groll = DoubleVar(root) # Roll in Degrees (z)
gyaw = DoubleVar(root) # Yaw in Degrees (x)
gax = StringVar(root) # Acceleration in X-axis
gay = StringVar(root) # Acceleration in Y-axis
gaz = StringVar(root) # Acceleration in Z-axis
ggx = StringVar(root) # Gyro in X-axis
ggy = StringVar(root) # Gyro in Y-axis
ggz = StringVar(root) # Gyro in Z-axis
gpressure = StringVar(root) # Pressure in mbar
galtitude = StringVar(root) # Altitude in meters
refresh_rate = .1 # viz refresh rate
g_last_draw = 0.0


# Parse the data from the subscriber
def imu_callback(data):
	quat = (data.orientation.x,
			data.orientation.y,
			data.orientation.z,
			data.orientation.w)

	euler = euler_from_quaternion(quat)

	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]

	# change from radian to degrees
	roll *= 180.0 / math.pi
	pitch  *= 180.0 / math.pi
	yaw   *= 180.0 / math.pi
	if yaw < 0: yaw   += 360.0  # Ensure yaw stays between 0 and 360

	groll.set('%2.2f' % roll)
	gpitch.set('%2.2f' % pitch)
	gyaw.set('%2.2f' % yaw)
	
	gax.set('%2.2f x' % data.linear_acceleration.x)
	gay.set('%2.2f y' % data.linear_acceleration.y)
	gaz.set('%2.2f z' % data.linear_acceleration.z)
	
	ggx.set('%2.2f x' % data.angular_velocity.x)
	ggy.set('%2.2f y' % data.angular_velocity.y)
	ggz.set('%2.2f z' % data.angular_velocity.z)

	redraw_viz()

def temp_callback(data):
	gtemp.set('%2.2f C' % data.temperature)


def press_callback(data):
	gpressure.set('%2.2f mbar' % data.fluid_pressure)


def alt_callback(data):	
	galtitude.set('%2.2f m' % data.data)

def redraw_viz():
	global g_last_draw
	if (rospy.Time.now().to_sec() > (refresh_rate + g_last_draw)):
		g_last_draw = rospy.Time.now().to_sec()
		# redraw imu box
		doDraw() 

def imu():

	rospy.init_node('imu_viz', anonymous=True)

	rospy.Subscriber('imu', Imu, imu_callback)
	rospy.Subscriber('sensors/temp', Temperature, temp_callback)
	rospy.Subscriber('sensors/pressure', FluidPressure, press_callback)
	rospy.Subscriber('sensors/alt', Float64, alt_callback)


# GUI stuff
root.title("ROS EM7180")

label = Label(root, text="Displaying data collected from EM7180 IMU")
label.grid(columnspan=5, sticky=W)
label.config(font=("Courier", 26))

label_ph2 = Label(root,  text="             ")
label_ph2.grid(row=1, column=2)
label_ph2.config(font=("Courier", 22))
label_ph3 = Label(root,  text="             ")
label_ph3.grid(row=1, column=3)
label_ph3.config(font=("Courier", 22))


label_pitchT = Label(root,  text="Pitch:")
label_pitchT.grid(row=3, sticky=E)
label_pitchT.config(font=("Courier", 22))

label_pitch = Label(root,  textvariable=gpitch)
label_pitch.grid(row=3, column=1)
label_pitch.config(font=("Courier", 22))

label_rollT = Label(root,  text="Roll:")
label_rollT.grid(row=4, sticky=E)
label_rollT.config(font=("Courier", 22))

label_roll = Label(root,  textvariable=groll)
label_roll.grid(row=4, column=1)
label_roll.config(font=("Courier", 22))

label_yawT = Label(root,  text="Yaw:")
label_yawT.grid(row=5, sticky=E)
label_yawT.config(font=("Courier", 22))

label_yaw = Label(root,  textvariable=gyaw)
label_yaw.grid(row=5, column=1)
label_yaw.config(font=("Courier", 22))

label_accT = Label(root,  text="Acceleration:")
label_accT.grid(row=6, sticky=E)
label_accT.config(font=("Courier", 22))

label_accX = Label(root,  textvariable=gax)
label_accX.grid(row=6, column=1)
label_accX.config(font=("Courier", 22))
label_accY = Label(root,  textvariable=gay)
label_accY.grid(row=6, column=2)
label_accY.config(font=("Courier", 22))
label_accZ = Label(root,  textvariable=gaz)
label_accZ.grid(row=6, column=3)
label_accZ.config(font=("Courier", 22))

label_gyroT = Label(root,  text="Gyro:")
label_gyroT.grid(row=7, sticky=E)
label_gyroT.config(font=("Courier", 22))

label_gyroX = Label(root,  textvariable=ggx)
label_gyroX.grid(row=7, column=1)
label_gyroX.config(font=("Courier", 22))
label_gyroY = Label(root,  textvariable=ggy)
label_gyroY.grid(row=7, column=2)
label_gyroY.config(font=("Courier", 22))
label_gyroZ = Label(root,  textvariable=ggz)
label_gyroZ.grid(row=7, column=3)
label_gyroZ.config(font=("Courier", 22))

label_tempT = Label(root,  text="Temperature:")
label_tempT.grid(row=8, sticky=E)
label_tempT.config(font=("Courier", 22))

label_temp = Label(root,  textvariable=gtemp)
label_temp.grid(row=8, column=1)
label_temp.config(font=("Courier", 22))

label_pressT = Label(root,  text="Pressure:")
label_pressT.grid(row=9, sticky=E)
label_pressT.config(font=("Courier", 22))

label_press = Label(root,  textvariable=gpressure)
label_press.grid(row=9, column=1)
label_press.config(font=("Courier", 22))

label_altT = Label(root,  text="Altitude:")
label_altT.grid(row=10, sticky=E)
label_altT.config(font=("Courier", 22))

label_alt = Label(root,  textvariable=galtitude)
label_alt.grid(row=10, column=1)
label_alt.config(font=("Courier", 22))

#Define Canvas for displaying 3D cube to visualize positioning in 3D space
cubeCanvas = Canvas(root, width=300, height=300)
cubeCanvas.grid(row=11, columnspan=2)

	# TODO: Change box to tractor
	# load the .gif image file
	#gif1 = PhotoImage(file='tractor.gif')

	# put gif image on canvas
	# pic's uper left corner (NW) on the canvas is at x=50 y=10
	#cubeCanvas.create_image(50, 10, image=gif1, anchor=NW)

def rotY(x, y, z, angle):
	#Rotates the point around the X axis by the given angle in degrees
	rad = math.radians(angle)
	cosa = math.cos(rad)
	sina = math.sin(rad)
	ny = y*cosa - z*sina
	nz = y*sina + z*cosa
	nx = x
	return nx, ny, nz

def rotX(x, y, z, angle):
	#Rotates the point around the Y axis by the given angle in degrees
	rad = math.radians(angle) #angle* math.pi/180
	cosa = math.cos(rad)
	sina = math.sin(rad)
	nz = z * cosa - x * sina
	nx = z * sina + x * cosa
	ny = y
	return nx, ny, nz

def rotZ(x, y, z, angle):
	#Rotates the point around the Z axis by the given angle in degrees
	rad = math.radians(angle)
	cosa = math.cos(rad)
	sina = math.sin(rad)
	nx = x * cosa - y * sina
	ny = x * sina + y * cosa
	nz = z
	return nx, ny, nz

def get2DPoint(x, y, z):
	#3D projections onto 2D plane
	win_width = 300
	win_height = 300
	fov = 256
	viewer_distance = 4
	
	factor = fov / (viewer_distance + z)
	nx = x * factor + win_width / 2
	ny = -y * factor + win_height / 2
	return nx, ny

def getPoint(x, y, z):
	# Do the whole point calculation
	nx, ny, nz = rotY(x, y, z, gpitch.get()) #gpitch.get()
	nx, ny, nz = rotX(nx, ny, nz, gyaw.get())#gyaw.get()
	nx, ny, nz = rotZ(nx, ny, nz, groll.get())
	nx, ny = get2DPoint(nx, ny, nz)
	
	return nx, ny

def doDraw():
	# The cube itself
	#c = cube. Fr,Le,Ri,Ba = Front-,Left-,Right-,Back- plane. L,R = Left-,Right- side of plane. t,b = top,bottom.
	#front=red. left=green. right=blue. back=yellow

	cFrLt = getPoint(-1, 1, -1)
	cFrRt = getPoint(1, 1, -1)
	cFrLb = getPoint(-1, -1, -1)
	cFrRb = getPoint(1, -1, -1)

	cLeLt = getPoint(-1, 1, 1)
	cLeRt = getPoint(-1, 1, -1)
	cLeLb = getPoint(-1, -1, 1)
	cLeRb = getPoint(-1, -1, -1)

	cRiLt = getPoint(1, 1, -1)
	cRiRt = getPoint(1, 1, 1)
	cRiLb = getPoint(1, -1, -1)
	cRiRb = getPoint(1, -1, 1)

	cBaLt = getPoint(1, 1, 1)
	cBaRt = getPoint(-1, 1, 1)
	cBaLb = getPoint(1, -1, 1)
	cBaRb = getPoint(-1, -1, 1)

	cubeCanvas.delete(ALL) #Called to refresh the canvas.

	cubeFront = cubeCanvas.create_polygon([cFrLt[0],cFrLt[1],cFrRt[0],cFrRt[1],cFrRb[0],cFrRb[1],cFrLb[0],cFrLb[1]], fill='', width=5, outline='red')

	cubeLeft = cubeCanvas.create_polygon([cLeLt[0],cLeLt[1],cLeRt[0],cLeRt[1],cLeRb[0],cLeRb[1],cLeLb[0],cLeLb[1]], fill='', width=5, outline='green')

	cubeRight = cubeCanvas.create_polygon([cRiLt[0],cRiLt[1],cRiRt[0],cRiRt[1],cRiRb[0],cRiRb[1],cRiLb[0],cRiLb[1]], fill='', width=5, outline='blue')

	cubeBack = cubeCanvas.create_polygon([cBaLt[0],cBaLt[1],cBaRt[0],cBaRt[1],cBaRb[0],cBaRb[1],cBaLb[0],cBaLb[1]], fill='', width=5, outline='yellow')#


if __name__ == '__main__':
    imu()

root.mainloop()


