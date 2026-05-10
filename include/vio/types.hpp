#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <variant>
#include <cstdint>


namespace vio{
    struct ImuMeasurement{
        int64_t timestamp;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
    };
    struct ImageMeasurement{
        int64_t timestamp;
        cv::Mat image;
    };
    using SensorMeasurement = std::variant<ImuMeasurement, ImageMeasurement>;
};
