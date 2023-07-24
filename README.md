# calibration_lidar

## 一.Docker镜像

本地链接权限 \
$ xhost +local: \
拉镜像 \
$ docker pull zhuoyuan0/calibration_lidar \
建立容器 \
$ docker run -it \ \
--privileged \ \
-v /run/udev:/run/udev  \ \
-v /dev/bus/usb:/dev/bus/usb \ \
-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \ \
-v "$XAUTH:$XAUTH" \ \
-v ~/Documents:/root/Documents \ \
--device-cgroup-rule='c 189:* rmw' \ \
-e "DISPLAY=$DISPLAY"  \ \
-e "QT_X11_NO_MITSHM=1" \ \
-e "XAUTHORITY=$XAUTH" \ \
--network host \ \
--shm-size 15G \ \
-p 5900:5900 \ \
--gpus all \ \
--name=calibration_lidar_v1 \ \
zhuoyuan0/calibration_lidar  /bin/bash

## 二.克隆项目

$ cd \
$ git clone https://github.com/zhuoyuan00/calibration_lidar.git \
$ cd calibration_lidar

## 三.编译

$ mkdir build && cd build \
$ cmake .. && make -j

## 四.运行

先将待配准点云拷贝到docker环境中/root/calibration_kit/test_data/lidar2lidar \
$ cd build \
$ ./calibration_kit \
选择点云文件
