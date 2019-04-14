# em7180

A simple python package to use em7180 IMU in ROS. Tested on Raspberry Pi 3B with ROS Kinetic.

# Wiring

Connect the em7180 to your Raspberry Pi

For the RPi SDA and SCL should have external pull-up 4.7kohm resistors (to 3.3V). 

Hardware setup:
EM7180 __________ RPi 3
3V3 _____________ 1 (3.3V)
SDA _____________ 3 (SDA)
SCL _____________ 5 (SCL)
GND _____________ 9 GND
INT _____________ ???


# Installation

    cd ~/catkin_ws/src
    git clone -b master https://github.com/droter/em7180_imu/
    cd ~/catkin_ws
    catkin build

# Usage

Python SMbus requires root access. It may therefore be required to run it as root:

    sudo su

You could however consider adding the user to the I2C usergroup to avoid running the package as root.

    adduser $USER i2c

To run the driver:

    rosrun em7180_imu imu_driver_node.py
    
To run the visualization:

    rosrun em7180_imu imu_viz_node.py

# Calibration

https://github.com/kriswiner/EM7180_SENtral_sensor_hub/wiki/F.--Magnetometer-and-Accelerometer-Calibration
    
# Documentation

Published Topics:

imu  (sensor_msgs/Imu)
imu/mag  (sensor_msgs/MagneticField)
sensor/temp  (sensor_msgs/Temperature)
sensor/pressure  (sensor_msgs/FluidPressure)
sensor/alt  (sensor_msgs/Float64)


## Angles
This project uses Tait-Bryan angles, commonly used in aircraft orientation.  In this coordinate system, the positive z-axis is down toward Earth.  Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.  Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.  Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.  These arise from the definition of the homogeneous rotation matrix constructed from q.  Tait-Bryan angles as well as Euler angles are non-commutative that is, the get the correct orientation the rotations must be applied in the correct order which for this configuration is yaw, pitch, and then roll.  For more information see this [Wikipedia article](http://en.wikipedia.org/wiki/Conversion_between_q_and_Euler_angles) which has additional links.


# Credits
The package uses some python scripts provided by simondlevy's [repository](https://github.com/simondlevy/EM7180).
The package uses some python scripts provided by vortexntnu's [repository](https://github.com/vortexntnu/em7180)

