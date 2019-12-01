# kitti_tracking_player

## Outline
This package is used to working with **KITTI Tracking** dataset, it can be used to parse **Tracking** dataset image groudtruth label and the **Tracklet** in raw data correspond to the same with the **Tracking** sequence.

## Changes
Compared with original **Kitti_player**, we add following features:

- Parsing the image detection label in **Tracking** dataset, add this information into `darknet_ros_msgs/ImageWithBBoxes` message.
- Parsing `tracklet.xml` file for offering 3D object tracking, this function is not perfect, and need to be improved.

## Reference
 * https://github.com/tomas789/kitti_player


## Node information
### Published Topics
 * /kitti/velo/pointcloud[sensor_msgs/PointCloud2]
* /kitti/oxts/gps [sensor_msgs/NavSatFix]
* /kitti/oxts/imu [sensor_msgs/Imu]
* /darknet_ros/image_with_bboxes [darknet_ros_msgs/ImageWithBBoxes]
* /viz/visualization_marker [visualization_msgs/Marker]
* /detection/object_array [iv_dynamicobject_msgs/ObjectArray]


