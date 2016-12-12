# smartek_camera ROS package

ROS drivers for [Smartek Vision](https://smartek.vision/home/) GigE Cameras. The following cameras have been tested and are known to work:

- Giganetix GC1291CP

## Installation instructions

The driver has been tested with ROS Kinetic on Ubuntu 16.04 64-bit.

### Prerequisites

Download the [Smartek GigEVision SDK](https://smartek.vision/header-menu/media-center/) and install it [following the official instructions](https://smartek.vision/fileadmin/SMARTEKVision_GigEVisionSDK_Linux_Readme.txt).

**Note:** The "Driver Installation" section of the SDK installation guide has to be repeated after every kernel update!

### Installation procedure

The package is still under development, i.e. it has not been released into official ROS distributions yet, so "install" it into your ROS workspace following the standard ROS procedure:

- clone this repo into the `src` subfolder of your workspace
- from your workspace root, invoke `catkin_make` or `catkin build` (if you are using catkin tools, which is highlly recommended)

## Testing the package

TODO...
