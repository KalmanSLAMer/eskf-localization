#               Using ESKF to fuse IMU and GPS data to achieve global     localization.

### The code is implemented base on the book "Quaterniond kinematics for the error-state Kalman filter"

## For formula derivation, please refer to：https://zhuanlan.zhihu.com/p/152662055  and https://blog.csdn.net/u011341856/article/details/114262451.

## 1.Dataset
https://epan-utbm.github.io/utbm_robocar_dataset/

## 2.Demo

![two_localization_result](https://github.com/KalmanSLAMer/eskf-localization/blob/master/doc/two_localization_result.png)

![localization_result](https://github.com/KalmanSLAMer/eskf-localization/blob/master/doc/localization_result.png)

## 3.Prerequisites

### 3.1 **Ubuntu** and **ROS**

**Ubuntu >= 18.04 (Ubuntu 16.04 is not supported)**

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 3.2 Nmea_navsat_driver

sudo apt-get install ros-melodic-nmea-navsat-driver libgps-dev

### 3.3 Mapviz

sudo apt-get install ros-$ROS_DISTRO-mapviz ros-$ROS_DISTRO-mapviz-plugins ros-$ROS_DISTRO-tile-map ros-$ROS_DISTRO-multires-image

## 4.Build

```
cd ~/catkin_ws/src
git clone https://github.com/KalmanSLAMer/eskf-localization.git
cd ..
catkin_make
source devel/setup.bash
```

## 5.Direct run

```
roslaunch imu_gps_localization imu_gps_localization.launch
rosbag play YOUR_DOWNLOADED.bag
```

## 6.How to use mapviz

how to How to set mapviz, please refer to [csdn blog](https://blog.csdn.net/weixin_41281151/article/details/114046438).

```
roslaunch mapviz mapviz.launch
```

## 7.TODO

  - [ ] add localization by fusing lidar and imu

  - [x] add normal deduction

  - [ ] add EKF

  - [ ] clean code

  - [x] Visualization via mapviz

    

## 8.Acknowledgments

Thanks for the book "Quaterniond kinematics for the error-state Kalman filter", [eskf_qk](https://github.com/ydsf16/imu_gps_localization), [eskf](https://github.com/zm0612/eskf-gps-imu-fusion), [mapviz](https://github.com/swri-robotics/mapviz), and [how to use mapviz](https://blog.csdn.net/weixin_41281151/article/details/114046438).



