# jaco-kinect-calib

ROS package to calibrate the Kinect Camera 📷 using a Kinova JACO arm 💪

This package was built under ROS Kinetic. The packages used for Kinova JACO are [here](https://github.com/nmssilva/kinova-ros), and for the Kinect are [here](https://github.com/ros-drivers/openni_camera).

<p align="center">
  <img src ="https://i.imgur.com/sySgmWt.png" />
</p>

# Setup

First of all, it is needed a workspace to compile the package:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```
Secondly, clone this repository into the `src` folder:

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/nmssilva/jaco-kinectic-calib.git
```
Now compile it.

```
$ cd ~/catkin_ws/
$ catkin_make
```
Make sure you also have the Kinova and Openni packages in the `src` folder.

Once everything is compiled, run the driver for the JACO and the Kinect. After you have the JACO and the Kinect running in your PC type this in your terminal:

```
$ rosrun jaco_kinect_calib jaco_kinect_calibration_node
```
This will run the calibration and will take a couple of seconds to complete. When the calibration is done a file named `camera_rgb_optical_frame.txt` will be created with something like this:

```
   x        y        z       yaw     pitch   roll
0.369727 0.167635 0.833969 0.003446 3.14159 0.53231
```

In the terminal run this command

```
rosrun tf static_transform_publisher x y z yaw pitch roll m1n6s200_link_base camera_rgb_optical_frame 100
```

Substitute the `x y z yaw pitch roll` with the values given in the file.

Now your camera is calibrated. Enjoy 👌

