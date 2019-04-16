# em7180

A simple python package to use em7180 IMU in ROS. Tested on Raspberry Pi 3B with ROS Kinetic.

https://www.tindie.com/products/onehorse/ultimate-sensor-fusion-solution-lsm6dsm-lis2md/

### Calibration

https://github.com/gregtomasch/EM7180_SENtral_Calibration

### Wiring

    Connect the em7180 to your Raspberry Pi

    For the RPi SDA and SCL should have external pull-up 4.7kohm resistors (to 3.3V). 

        Hardware setup:
        EM7180 __________ RPi 3
        3V3 _____________ 1 (3.3V)
        SDA _____________ 3 (SDA)
        SCL _____________ 5 (SCL)
        GND _____________ 9 GND
        INT _____________ ???


### Installation

    cd ~/catkin_ws/src
    git clone -b master https://github.com/droter/em7180_imu/
    cd ~/catkin_ws
    catkin build

### Usage

    Python SMbus requires root access. It may therefore be required to run it as root:

        sudo su

    You could however consider adding the user to the I2C usergroup to avoid running the package as root.

        adduser $USER i2c

    To run the driver:

        roslaunch em7180_imu imu_driver.launch
        
    To run the visualization:

        roslaunch em7180_imu imu_viz.launch
    

### Documentation

Published Topics:

    imu/data  (sensor_msgs/Imu)
    imu/data_raw  (sensor_msgs/Imu) No orientation 
    imu/mag  (sensor_msgs/MagneticField)
    sensor/temp  (sensor_msgs/Temperature)
    sensor/pressure  (sensor_msgs/FluidPressure)
    sensor/alt  (sensor_msgs/Float64)

### Angles
    The ROS standard for IMU data http://www.ros.org/reps/rep-0145.html
    The ROS standard for Coordinate Conventions http://www.ros.org/reps/rep-0103.html

    	This data is prepared to be fused with Robot_Localization
            Yaw is positive when rotated counter clockwise.
            Pitch is positive when nose is down.
            Roll is positive when left side is up.
            
            Yaw is zero when pointing East

### Mounting the imu on your robot



### Credits
    The package uses some python scripts provided by simondlevy's [repository](https://github.com/simondlevy/EM7180)
    The package uses some python scripts provided by vortexntnu's [repository](https://github.com/vortexntnu/em7180)

