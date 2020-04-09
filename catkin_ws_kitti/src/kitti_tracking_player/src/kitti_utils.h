/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-09 22:36:20
 * @LastEditTime: 2020-04-10 00:04:10
 * @Description: Some utilities function and classes for loading and parsing KITTI dataset
 * @References: 
 */
#pragma once

// C++
#include <string>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI KittiPoint;
typedef pcl::PointCloud<KittiPoint> KittiPointCloud;
typedef KittiPointCloud::Ptr KittiPointCloudPtr;

namespace kitti_utils {

KittiPointCloudPtr GetPointCloud()

class Calibration {
public:
  Calibration(const std::string& calib_file_path) {
    
  }
};

} // namespace kitti_utils