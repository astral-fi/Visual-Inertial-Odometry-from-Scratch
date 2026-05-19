#include "types.hpp"
#include <cmath>

namespace vio {
    class IMUPreintegration {
    private:
        PreintegratedIMU preintegrated_imu;
    public:
        IMUPreintegration() {
            preintegrated_imu.accumulated_time = 0;
            preintegrated_imu.accumulated_position = Eigen::Vector3d::Zero();
            preintegrated_imu.accumulated_velocity = Eigen::Vector3d::Zero();
            preintegrated_imu.accumulated_rotation = Eigen::Quaterniond::Identity();
            preintegrated_imu.gyro_bias = Eigen::Vector3d::Zero();
            preintegrated_imu.accel_bias = Eigen::Vector3d::Zero();
        } 
        void reset(const Eigen::Vector3d& gyro_bias, const Eigen::Vector3d& accel_bias) {
            preintegrated_imu.accumulated_time = 0;
            preintegrated_imu.accumulated_position = Eigen::Vector3d::Zero();
            preintegrated_imu.accumulated_velocity = Eigen::Vector3d::Zero();
            preintegrated_imu.accumulated_rotation = Eigen::Quaterniond::Identity();
            preintegrated_imu.gyro_bias = gyro_bias;
            preintegrated_imu.accel_bias = accel_bias;
        }
        void integrate(const ImuMeasurement& imu_measurement){
            double dt = (imu_measurement.timestamp - preintegrated_imu.accumulated_time) * 1e-9;
            Eigen::Vector3d accel = imu_measurement.accel - preintegrated_imu.accel_bias;
            Eigen::Vector3d gyro = imu_measurement.gyro - preintegrated_imu.gyro_bias;
            preintegrated_imu.accumulated_time = imu_measurement.timestamp;
            preintegrated_imu.accumulated_position += preintegrated_imu.accumulated_velocity * dt + 0.5 * preintegrated_imu.accumulated_rotation.toRotationMatrix() * accel * dt * dt;
            preintegrated_imu.accumulated_velocity += preintegrated_imu.accumulated_rotation.toRotationMatrix() * accel * dt;
            Eigen::Matrix3d delta_rotation = expMap(gyro * dt);
            preintegrated_imu.accumulated_rotation = Eigen::Quaterniond(delta_rotation) * preintegrated_imu.accumulated_rotation;     
        
        }
        const PreintegratedIMU& getPreintegratedIMU() const {
            return preintegrated_imu;
        }
    private:
        Eigen::Matrix3d expMap(const Eigen::Vector3d& omega) {
            double theta = omega.norm();
            if (theta < 1e-5) {
                return Eigen::Matrix3d::Identity() + skewSymmetric(omega);
            } else {
                Eigen::Matrix3d K = skewSymmetric(omega / theta);
                return Eigen::Matrix3d::Identity() + std::sin(theta) * K + (1 - std::cos(theta)) * K * K;
            }
        }
        Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) {
            Eigen::Matrix3d skew;
            skew << 0, -v.z(), v.y(),
                    v.z(), 0, -v.x(),
                    -v.y(), v.x(), 0;
            return skew;
        }
    };
}