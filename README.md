# birl_ensenso
This package acts as a ROS driver of ensenso 3D camera, especially for usage of N10-304 in Biomimetics Robotics lab at Guangdong University of Technology.

## Installation
### Install EnsensoSDK and uEyeDriver

Go to the [ensenso offical website](http://www.ensenso.com/support/sdk-download/) and follow the instructions

### Install birl_ensenso

Go to you ROS workspace
```{bash}
cd ~/catkin_ws/src
```

Clone birl_ensenso
```{bash}
git clone https://github.com/birlrobotics/birl_ensenso.git
```

## Usage

To test whether EnsensoSDK and uEyeDriver are installed properly, please connect the camera,open a terminal window and type the following command
```
nxView
```
This will start a official tool. Select the camera and click open to check if it works normally.

For starting the camera ROS driver, type following command.
```
roslaunch ensenso ensenso_bringup.launch
```
To change camera parameters such as ROI, depth range, stereo-matching related params, etc., please use nxView to modify these parameters and export the new configuration files(.json format). Input the path of those new files to the camera configuration arguments in launch file will apply the new changes.

Several ROS services are provided. Please look into the srv folder for details.

To get more infos about how to use EnsensoSDK, please read through the ensenso [online manual](http://www.ensenso.com/manual/)


