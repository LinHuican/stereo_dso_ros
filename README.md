# STEREO_DSO_ROS

这是stereo_dso的ROS接口，可以使用EuRoC数据集运行地平线开发的stereo_dso，当然也可以用自己的立体摄像机。

关于stereo_dso: https://github.com/HorizonAD/stereo_dso

# 1. Installation

## 1.1. 安装stereo_dso

根据如下链接的描述进行安装：

https://github.com/HorizonAD/stereo_dso

## 1.2. 安装stereo_dso_ros

cd ~/catkin_ws/src

git clone https://github.com/LinHuican/stereo_dso_ros

cd ..

catkin_make

# 2 使用示例

rosrun stereo_dso_ros stero_dso_ros calib=/home/huicanlin/catkin_ws/src/stereo_dso_ros/examples/camera_euroc.txt preset=0 mode=1

rosbag play --pause ./Downloads/Dataset/ETH/V1_01_easy.bag

# 3 注意

非常简陋的代码，仍存在很多不完善的地方，抛砖引玉，希望能够得到高手指点。此外，stereo_dso运行EuRoC数据集，应该还需要调整很多参数，目前效果不好。

# 4 License

