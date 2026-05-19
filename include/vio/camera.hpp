#include "types.hpp"
#include <yaml-cpp/yaml.h>

namespace vio{
    class Camera{
        private:
            CameraIntrinsics intrinsics;
            cv::Mat K;
            cv::Mat distCoeffs;
        public:
            Camera() = default;
            Camera(const std::string &calibration_file_path){
                YAML::Node calib = YAML::LoadFile(calibration_file_path);
                intrinsics.width = calib["resolution"][0].as<int>();
                intrinsics.height = calib["resolution"][1].as<int>();
                intrinsics.fx = calib["intrinsics"][0].as<double>();
                intrinsics.fy = calib["intrinsics"][1].as<double>();
                intrinsics.cx = calib["intrinsics"][2].as<double>();
                intrinsics.cy = calib["intrinsics"][3].as<double>();    
                intrinsics.k1 = calib["distortion_coefficients"][0].as<double>();
                intrinsics.k2 = calib["distortion_coefficients"][1].as<double>();
                intrinsics.p1 = calib["distortion_coefficients"][2].as<double>();
                intrinsics.p2 = calib["distortion_coefficients"][3].as<double>();

                K = (cv::Mat_<double>(3, 3) << intrinsics.fx, 0, intrinsics.cx,
                                                 0, intrinsics.fy, intrinsics.cy,
                                                 0, 0, 1);
                distCoeffs = (cv::Mat_<double>(1, 4) << intrinsics.k1, intrinsics.k2, intrinsics.p1, intrinsics.p2);

            }

            std::vector<cv::Point2f> undistortPoints(const std::vector<cv::Point2f>& distorted_points) const {
                std::vector<cv::Point2f> undistorted_points;
                cv::undistortPoints(distorted_points, undistorted_points, K, distCoeffs);
                return undistorted_points;
            }

            const CameraIntrinsics& getIntrinsics() const {
                return intrinsics;
            }
    };
}