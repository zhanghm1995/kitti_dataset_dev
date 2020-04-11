/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-09 22:37:19
 * @LastEditTime: 2020-04-11 16:10:18
 * @Description: Some utilities function and classes for loading and parsing KITTI dataset
 * @References: 
 */

#include "kitti_utils.h"

#include <fstream>

#include "utils/string_utils.h"

namespace kitti_utils {

using std::cout;
using std::endl;

Calibration::Calibration(const std::string& calib_file_path) {
  LoadFile2Map(calib_file_path);
  
	auto P2_temp = calib_params_["P2:"];
	P2_ = Eigen::Map<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor> >(P2_temp.data());

	auto R0_Rect_temp = calib_params_["R_rect"];
	R_Rect_0_ = Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> >(R0_Rect_temp.data());

	auto Velo2Cam_temp = calib_params_["Tr_velo_cam"];
	Velo2Cam_ = Eigen::Map<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor> >(Velo2Cam_temp.data());

  cout<<"P2 = "<<P2_<<endl;
  cout<<"R_Rect_0 = "<<R_Rect_0_<<endl;
  cout<<"Velo2Cam = "<<Velo2Cam_<<endl;
}
	
void Calibration::LoadFile2Map(const std::string& calib_file_path) {
  std::ifstream file(calib_file_path);
  if (!file.is_open()) {
    throw std::logic_error("[Calibration::LoadFile2Map] Failed to read calibration file!");
  }

  std::string line;
  while (std::getline(file, line)) {
    std::vector<std::string> line_content;
    Split(rtrim(line), line_content, " ");
    std::vector<std::string> calib_param_string(line_content.begin() + 1, line_content.end());
    std::vector<float> calib_param(calib_param_string.size());
    std::transform(calib_param_string.begin(), calib_param_string.end(), calib_param.begin(), [](const std::string& val) { return std::atof(val.c_str()); });
    calib_params_[line_content[0]] = calib_param;
  }
}

Eigen::MatrixXf Calibration::Cartesian2Homogenous(const Eigen::MatrixXf& pts_3d) {
  if (pts_3d.size() == 4) {
    return pts_3d;
  }
  Eigen::MatrixXf temp = Eigen::MatrixXf::Ones(1, pts_3d.cols());
  Eigen::MatrixXf ret;
  ret << pts_3d,
               temp;
  return ret;
}

Eigen::MatrixXf Calibration::ProjectVelo2Ref(const Eigen::MatrixXf& pts_3d_velo) {
  Eigen::MatrixXf pts_3d_homo = Cartesian2Homogenous(pts_3d_velo);
  return Velo2Cam_ * pts_3d_homo;
}
  
Eigen::MatrixXf Calibration::ProjectRef2Rect(const Eigen::MatrixXf& pts_3d_ref) {
  return R_Rect_0_ * pts_3d_ref;
}

Eigen::MatrixXf Calibration::ProjectVelo2Rect(const Eigen::MatrixXf& pts_3d_velo) {
  Eigen::MatrixXf velo_pts_in_ref = ProjectVelo2Ref(pts_3d_velo);
  return ProjectRef2Rect(velo_pts_in_ref);
}

Eigen::MatrixXf Calibration::ProjectRect2Image(const Eigen::MatrixXf& pts_3d_rect) {
  Eigen::MatrixXf pts_3d_homo = Cartesian2Homogenous(pts_3d_rect);
  Eigen::MatrixXf pts_image = P2_ * pts_3d_homo;
  pts_image.array().rowwise() /= pts_image.row(2).array();
  return pts_image.topRows(2);
}

Eigen::MatrixXf Calibration::GetVelo2ImageMatrix() const {
  Eigen::MatrixXf temp1  = R_Rect_0_ * Velo2Cam_;
  Eigen::Matrix4f temp2 = Eigen::Matrix4f::Zero();
  temp2(3, 3) = 1.0;
  temp2.topLeftCorner(3, 4) = temp1;
  Eigen::MatrixXf ret = P2_ * temp2;
  return ret;
}

}  // namespace kitti_utils