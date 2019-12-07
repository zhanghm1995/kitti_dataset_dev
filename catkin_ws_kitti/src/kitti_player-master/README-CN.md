# kitti_player

## 介绍

该ROS项目的主要目的是能够将KITTI raw_data数据以ROS中常用的消息类型以话题的形式发布出来。

发布的话题类型包括：Color/Grayscale images, Velodyne scan as PCL, sensor_msgs/Imu Message, GPS as sensor_msgs/NavSatFix Message). 


## 节点和发布话题内容

Node [/kitti_player]
Publications: 
* /kitti_player/GT_RTK [visualization_msgs/MarkerArray]
* /kitti_player/color/left/camera_info [sensor_msgs/CameraInfo]
* /kitti_player/color/left/image_rect [sensor_msgs/Image]
* /kitti_player/color/left/image_rect/compressed [sensor_msgs/CompressedImage]
* /kitti_player/color/left/image_rect/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/color/left/image_rect/compressed/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/color/left/image_rect/compressedDepth [sensor_msgs/CompressedImage]
* /kitti_player/color/left/image_rect/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/color/left/image_rect/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/color/left/image_rect/theora [theora_image_transport/Packet]
* /kitti_player/color/left/image_rect/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/color/left/image_rect/theora/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/color/right/camera_info [sensor_msgs/CameraInfo]
* /kitti_player/color/right/image_rect [sensor_msgs/Image]
* /kitti_player/color/right/image_rect/compressed [sensor_msgs/CompressedImage]
* /kitti_player/color/right/image_rect/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/color/right/image_rect/compressed/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/color/right/image_rect/compressedDepth [sensor_msgs/CompressedImage]
* /kitti_player/color/right/image_rect/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/color/right/image_rect/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/color/right/image_rect/theora [theora_image_transport/Packet]
* /kitti_player/color/right/image_rect/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/color/right/image_rect/theora/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/grayscale/left/camera_info [sensor_msgs/CameraInfo]
* /kitti_player/grayscale/left/image_rect [sensor_msgs/Image]
* /kitti_player/grayscale/left/image_rect/compressed [sensor_msgs/CompressedImage]
* /kitti_player/grayscale/left/image_rect/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/grayscale/left/image_rect/compressed/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/grayscale/left/image_rect/compressedDepth [sensor_msgs/CompressedImage]
* /kitti_player/grayscale/left/image_rect/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/grayscale/left/image_rect/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/grayscale/left/image_rect/theora [theora_image_transport/Packet]
* /kitti_player/grayscale/left/image_rect/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/grayscale/left/image_rect/theora/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/grayscale/right/camera_info [sensor_msgs/CameraInfo]
* /kitti_player/grayscale/right/image_rect [sensor_msgs/Image]
* /kitti_player/grayscale/right/image_rect/compressed [sensor_msgs/CompressedImage]
* /kitti_player/grayscale/right/image_rect/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/grayscale/right/image_rect/compressed/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/grayscale/right/image_rect/compressedDepth [sensor_msgs/CompressedImage]
* /kitti_player/grayscale/right/image_rect/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/grayscale/right/image_rect/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/grayscale/right/image_rect/theora [theora_image_transport/Packet]
* /kitti_player/grayscale/right/image_rect/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
* /kitti_player/grayscale/right/image_rect/theora/parameter_updates [dynamic_reconfigure/Config]
* /kitti_player/hdl64e [sensor_msgs/PointCloud2]
* /kitti_player/oxts/gps [sensor_msgs/NavSatFix]
* /kitti_player/oxts/gps_initial [sensor_msgs/NavSatFix]
* /kitti_player/oxts/imu [sensor_msgs/Imu]
* /kitti_player/preprocessed_disparity [stereo_msgs/DisparityImage]
* /rosout [rosgraph_msgs/Log]

Subscriptions: 
* /kitti_player/synch [unknown type]

Services: 
* /kitti_player/color/left/image_rect/compressed/set_parameters
* /kitti_player/color/left/image_rect/compressedDepth/set_parameters
* /kitti_player/color/left/image_rect/theora/set_parameters
* /kitti_player/color/right/image_rect/compressed/set_parameters
* /kitti_player/color/right/image_rect/compressedDepth/set_parameters
* /kitti_player/color/right/image_rect/theora/set_parameters
* /kitti_player/get_loggers
* /kitti_player/grayscale/left/image_rect/compressed/set_parameters
* /kitti_player/grayscale/left/image_rect/compressedDepth/set_parameters
* /kitti_player/grayscale/left/image_rect/theora/set_parameters
* /kitti_player/grayscale/right/image_rect/compressed/set_parameters
* /kitti_player/grayscale/right/image_rect/compressedDepth/set_parameters
* /kitti_player/grayscale/right/image_rect/theora/set_parameters
* /kitti_player/set_logger_level




## 运行

先设置`kittiplayer_standalone.launch`中的`<arg name="directory"`中的`default`为你电脑中的KITTI raw_data的路径（要具体到哪一天的数据），例如设置为

`Datasets/KITTI/raw_data/2011_09_26/2011_09_26_drive_0005_sync/`。

然后运行如下命令：

```xml
roslaunch kitti_player kittiplayer_standalone.launch
```
```
Allowed options:
help           h    help message
directory      d    *required* - path to the kitti dataset Directory
frequency      f    set replay Frequency
all            a    replay All data
velodyne       v    replay Velodyne data
gps            g    replay Gps data
imu            i    replay Imu data
grayscale      G    replay Stereo Grayscale images
color          C    replay Stereo Color images
viewer         V    enable image viewer
timestamps     T    use KITTI timestamps
stereoDisp     s    use pre-calculated disparities
viewDisp       D    view loaded disparity images
frame          F    start playing at frame...
gpsPoints      p    publish GPS/RTK markers to RVIZ, having reference frame as <reference_frame> [example: -p map]
synchMode      S    Enable Synch mode (wait for signal to load next frame [std_msgs/Bool "data: true"]

kitti_player needs a directory tree like the following:
└── 2011_09_26_drive_0001_sync
    ├── image_00              
    │   └── data              
    │   └ timestamps.txt      
    ├── image_01              
    │   └── data              
    │   └ timestamps.txt      
    ├── image_02              
    │   └── data              
    │   └ timestamps.txt      
    ├── image_03              
    │   └── data              
    │   └ timestamps.txt      
    ├── oxts                  
    │   └── data              
    │   └ timestamps.txt      
    ├── velodyne_points       
    │   └── data              
    │     └ timestamps.txt    
    └── calib_cam_to_cam.txt  
```
另：

`my_kitti_player.cpp`为在原来`kitti_player.cpp`基础上修改的，主要是添加了点云往图像上投影的功能，运行命令为：

```xml
roslaunch kitti_player my_kittiplayer_standalone.launch
```

**参数含义：**

```
Allowed options:
help           h    help message
directory      d    *required* - path to the kitti dataset Directory
frequency      f    set replay Frequency
all            a    replay All data
velodyne       v    replay Velodyne data
gps            g    replay Gps data
imu            i    replay Imu data
grayscale      G    replay Stereo Grayscale images
color          C    replay Stereo Color images
viewer         V    enable image viewer
timestamps     T    use KITTI timestamps
stereoDisp     s    use pre-calculated disparities
viewDisp       D    view loaded disparity images
frame          F    start playing at frame...
gpsPoints      p    publish GPS/RTK markers to RVIZ, having reference frame as <reference_frame> [example: -p map]
synchMode      S    Enable Synch mode (wait for signal to load next frame [std_msgs/Bool "data: true"]

kitti_player needs a directory tree like the following:
└── 2011_09_26_drive_0001_sync
    ├── image_00              
    │   └── data              
    │   └ timestamps.txt      
    ├── image_01              
    │   └── data              
    │   └ timestamps.txt      
    ├── image_02              
    │   └── data              
    │   └ timestamps.txt      
    ├── image_03              
    │   └── data              
    │   └ timestamps.txt      
    ├── oxts                  
    │   └── data              
    │   └ timestamps.txt      
    ├── velodyne_points       
    │   └── data              
    │     └ timestamps.txt    
    └── calib_cam_to_cam.txt  
```



