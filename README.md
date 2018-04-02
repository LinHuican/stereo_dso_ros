# STEREO_DSO_ROS

This is a minimal example of how to integrate STEREO DSO from a ROS project, and run it on real-time input data. You can test it with EuRoC and KITTI Dataset(rosbag), or you may run it with your stereo camera(ROS API).

Detail about STEREO DSO: https://github.com/HorizonAD/stereo_dso

Assume that you are familiar with ROS. If you are not familiar with ROS, you can refer to the following link to install and learn how to use ROS.

Installation instructions:

http://wiki.ros.org/indigo/Installation

ROS Tutorials:

http://wiki.ros.org/ROS/Tutorials


# 1. Installation

## 1.1. Install stereo_dso

Install the stereo_dso as the instruction of the following link:

https://github.com/HorizonAD/stereo_dso

## 1.2. Install stereo_dso_ros


cd ~/catkin_ws/src

git clone https://github.com/LinHuican/stereo_dso_ros

cd ..

catkin_make

# 2 Examples

### KITTI dataset

rosrun stereo_dso_ros stero_dso_ros calib=/home/huicanlin/catkin_ws/src/stereo_dso_ros/examples/camera_kitti.txt preset=0 mode=1
rosrun stereo_dso_ros stero_dso_ros calib=/home/huicanlin/catkin_ws/src/stereo_dso_ros/examples/camera_kitti.txt preset=0 mode=1 /cam0/image_raw:=/kitti_stereo/left/image_rect /cam1/image_raw:=kitti_stereo/right/image_rect

rosbag play --pause ~/Downloads/Dataset/KITTI/kitti_00.bag

result:

/home/huicanlin/catkin_ws/src/stereo_dso_ros/examples/stereo_dso_ros_kitti_00.png

### EuRoC dataset(Todo:something wrong!)

rosrun stereo_dso_ros stero_dso_ros calib=/home/huicanlin/catkin_ws/src/stereo_dso_ros/examples/camera_euroc.txt preset=0 mode=1

rosbag play --pause ~/Downloads/Dataset/ETH/V1_01_easy.bag

# 3 Others

If you have any trouble installing or running STEREO DSO ROS, contact the authors.

# 4 License

