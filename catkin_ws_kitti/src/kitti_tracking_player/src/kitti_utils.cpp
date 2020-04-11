/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-04-09 22:37:19
 * @LastEditTime: 2020-04-11 08:15:46
 * @Description: Some utilities function and classes for loading and parsing KITTI dataset
 * @References: 
 */

#include "kitti_utils.h"

#include <fstream>

#include "utils/string_utils.h"

namespace kitti_utils {

Calibration::Calibration(const std::string& calib_file_path) {
	auto P2_temp = calib_params_["P2:"];
	P2_ = Eigen::Map<const Eigen::Matrix<float, 3, 4, Eigen::RowMajor> >(P2_temp.data());

	auto R0_Rect_temp = calib_params_["R_rect"];
	R_Rect_0_ = Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> >(R0_Rect_temp.data());

	auto Velo2Cam_temp = calib_params_["Tr_velo_cam"];
	Velo2Cam_ = Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> >(Velo2Cam_temp.data());
}
	
void Calibration::LoadFile2Map(const std::string& calib_file_path) {
  std::ifstream file(calib_file_path);
  if (!file.is_open()) {
    throw std::logic_error("[Calibration::LoadFile2Map] Failed to read calibration file!");
  }

  std::string line;
  while (std::getline(file, line)) {
    std::cout << line << std::endl;
    std::vector<std::string> line_content;
    Split(rtrim(line), line_content, " ");
    std::vector<std::string> calib_param_string(line_content.begin() + 1, line_content.end());
    std::vector<float> calib_param(calib_param_string.size());
    std::transform(calib_param_string.begin(), calib_param_string.end(), calib_param.begin(), [](const std::string& val) { return std::atof(val.c_str()); });
    calib_params_[line_content[0]] = calib_param;
  }
}

}  // namespace kitti_utils