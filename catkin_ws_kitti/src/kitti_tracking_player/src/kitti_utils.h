/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-09 22:36:20
 * @LastEditTime: 2020-04-11 08:15:08
 * @Description: Some utilities function and classes for loading and parsing KITTI dataset
 * @References: 
 */
#pragma once

// C++
#include <string>
#include <map>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI KittiPoint;
typedef pcl::PointCloud<KittiPoint> KittiPointCloud;
typedef KittiPointCloud::Ptr KittiPointCloudPtr;

namespace kitti_utils {

class Calibration {
public:
  typedef std::map<std::string, std::vector<float> > FileContentMap;
  Calibration(const std::string& calib_file_path);

protected:
  void LoadFile2Map(const std::string& calib_file_path);

protected:
  FileContentMap calib_params_;
  Eigen::MatrixXf P0_, P1_, P2_, P3_;
  Eigen::MatrixXf R_Rect_0_;
  Eigen::MatrixXf Velo2Cam_, Cam2Velo_;
};

} // namespace kitti_utils