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
    struct CameraIntrinsics{
        int width;
        int height;
        double fx;
        double fy;
        double cx;
        double cy;
        double k1, k2, p1, p2;
    };
    struct TrackedFeature{
        int id;
        cv::Point2f position;
        cv::Point2f normalized_position;
    };
    struct TrackedFrame{
        int64_t timestamp;
        std::vector<TrackedFeature> features;
    };
    struct PreintegratedIMU{
        int64_t accumulated_time;
        Eigen::Vector3d accumulated_position;
        Eigen::Vector3d accumulated_velocity;
        Eigen::Quaterniond accumulated_rotation;
        Eigen::Vector3d gyro_bias;
        Eigen::Vector3d accel_bias;
    };
    struct State{
        int64_t timestamp;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Quaterniond orientation;
        Eigen::Vector3d accel_bias;
        Eigen::Vector3d gyro_bias;

    };
    struct OptimizationState{
        double position[3];
        double velocity[3];
        double quaternions[4];
        double gyro_bias[3];
        double accel_bias[3];

        void fromVioState(const vio::State& viostate){
            std::copy(&viostate.position[0], &viostate.position[0] + 3, position);
            std::copy(&viostate.velocity[0], &viostate.velocity[0] + 3, velocity);
            std::copy(&viostate.orientation.coeffs()[0], &viostate.orientation.coeffs()[0] + 4, quaternions);
            std::copy(&viostate.gyro_bias[0], &viostate.gyro_bias[0] + 3, gyro_bias);
            std::copy(&viostate.accel_bias[0], &viostate.accel_bias[0] + 3, accel_bias);
        }
        void toVioState(vio::State& viostate){
            std::copy(position, position + 3, &viostate.position[0]);
            std::copy(velocity, velocity + 3, &viostate.velocity[0]);
            std::copy(quaternions, quaternions + 4, &viostate.orientation.coeffs()[0]);
            std::copy(gyro_bias, gyro_bias + 3, &viostate.gyro_bias[0]);
            std::copy(accel_bias, accel_bias + 3, &viostate.accel_bias[0]);
        }
    };
    using SensorMeasurement = std::variant<ImuMeasurement, ImageMeasurement>;
};
