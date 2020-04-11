/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-09 22:36:20
 * @LastEditTime: 2020-04-11 16:13:46
 * @Description: Some utilities function and classes for loading and parsing KITTI dataset
 * @References: 
 */
#pragma once

// C++
#include <string>
#include <fstream>
#include <map>
#include <dirent.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

typedef pcl::PointXYZI KittiPoint;
typedef pcl::PointCloud<KittiPoint> KittiPointCloud;
typedef KittiPointCloud::Ptr KittiPointCloudPtr;

namespace kitti_utils {

/**
 * @brief This function will find all files in specified directories, including hidden files
 *                Assume no subdirectory in dir_name folder
 **/  
inline int ListFilesInDirectory(const std::string& dir_name) {
  DIR* dir;
  struct dirent* ent;

  int total_files = 0;
  if ((dir = opendir(dir_name.c_str())) != NULL) {
    // print all the files and directories within directory
    while ((ent = readdir(dir)) != NULL) {
      // skip . & ..
      if (strlen(ent->d_name) > 2) {
        ++total_files;
      }
    }
    closedir(dir);
  } else {
    // cannot open the directory
    perror("Failed to open directory");
    return -1;
  }
  return total_files;
}

inline bool ReadVeloPoints(const std::string& velo_bin_path, KittiPointCloud& point_cloud) {
  std::fstream input(velo_bin_path.c_str(), std::ios::in | std::ios::binary);
  if (!input.good()) {
    std::cout<<"[ReadVeloPoints] Could not read file: "<<velo_bin_path<<std::endl;
    return false;
  } else {
    input.seekg(0, std::ios::beg);
    int i;
    for (i = 0; input.good() && !input.eof(); i++) {
      KittiPoint point;
      input.read((char*)&point.x, 3 * sizeof(float));
      input.read((char*)&point.intensity, sizeof(float));
      point_cloud.push_back(point);
    }
    input.close();
    return true;
  }
}

/**
 * @brief KITTI Tracking dataset calibration parameters manage class
 */ 
class Calibration {
public:
  typedef std::map<std::string, std::vector<float> > FileContentMap;

  Calibration() = default;
  
  Calibration(const std::string& calib_file_path);

  Eigen::MatrixXf R_Rect_0() const { return R_Rect_0_; }

  Eigen::MatrixXf P2() const { return P2_; }

  Eigen::MatrixXf Velo2Cam() const { return Velo2Cam_; }

  /**
   * @brief Get the 3x4 from 3d velodyne coordinate to 2d image coodinates transformation matrix
   */ 
  Eigen::MatrixXf GetVelo2ImageMatrix() const;

  /**
   * @brief From cartesian coordinate to homogenous coordinate, that's add 1 in the end
   */ 
  static Eigen::MatrixXf Cartesian2Homogenous(const Eigen::MatrixXf& pts_3d);

  /// 3d to 3d
  Eigen::MatrixXf ProjectVelo2Ref(const Eigen::MatrixXf& pts_3d_velo);
  
  Eigen::MatrixXf ProjectRef2Rect(const Eigen::MatrixXf& pts_3d_ref);

  Eigen::MatrixXf ProjectVelo2Rect(const Eigen::MatrixXf& pts_3d_velo);

  // 3d to 2d
  Eigen::MatrixXf ProjectRect2Image(const Eigen::MatrixXf& pts_3d_rect);

protected:
  void LoadFile2Map(const std::string& calib_file_path);

protected:
  FileContentMap calib_params_;
  Eigen::MatrixXf P0_, P1_, P2_, P3_;
  Eigen::MatrixXf R_Rect_0_;
  Eigen::MatrixXf Velo2Cam_, Cam2Velo_;
};

} // namespace kitti_utils