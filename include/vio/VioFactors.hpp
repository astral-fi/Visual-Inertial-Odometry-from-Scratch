#pragma once 
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "types.hpp"

namespace vio{
    struct ReprojectionFactor{
        ReprojectionFactor(double u, double v, double weight) : obs_u(u), obs_v(v), weight(weight) {}
        

        template<typename T>
        bool operator()(const T* const pose, const T* const point, T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pose_wc(pose); 
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> point_wc(point); 
            Eigen::Quaternion<T> q_wc(pose[3], pose[4], pose[5], pose[6]); // Assuming the quaternion is stored as (w, x, y, z)
        
            Eigen::Matrix<T, 3, 1> point_cc = q_wc.inverse() * (point_wc - pose_wc); // Transform point to camera frame
            residuals[0] = weight * (point_cc.x() / point_cc.z() - T(obs_u)); // Reprojection error in x
            residuals[1] = weight * (point_cc.y() / point_cc.z() - T(obs_v)); // Reprojection error in y
            return true;
        }

        static ceres::CostFunction* Create(const double u, const double v, const double weight) {
            return new ceres::AutoDiffCostFunction<ReprojectionFactor, 2, 7, 3>(
                new ReprojectionFactor(u, v, weight));
        }

        double obs_u, obs_v, weight;
    };

    struct ImuFactor{
        ImuFactor(const vio::ImuMeasurement& measurment, const Eigen::Vector3d& g): m(measurement), gravity(g) {}

        template<typename T>
        bool operator()(const T* const state_i, const T* const vel_i, const T* const bias_i, const T* const state_j, const T* const vel_j, T* residuals) const {
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pos_i(state_i); 
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> pos_j(state_j); 
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> vel_i(vel_i); 
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> vel_j(vel_j); 
            Eigen::Quaternion<T> q_i(state_i[3], state_i[4], state_i[5], state_i[6]); // Assuming the quaternion is stored as (w, x, y, z)
            Eigen::Quaternion<T> q_j(state_j[3], state_j[4], state_j[5], state_j[6]); // Assuming the quaternion is stored as (w, x, y, z)
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> bias_i(bias_i); 

            T dt = T(m.timestamp - state_i[0]); // Assuming the timestamp is stored in the first element of the state
            T dt2 = dt * dt;

            // Predicted position and velocity at time j based on state i and IMU measurements
            Eigen::Matrix<T, 3, 1> predicted_pos_j = pos_i + vel_i * dt + T(0.5) * (gravity - bias_i) * dt2;
            Eigen::Matrix<T, 3, 1> predicted_vel_j = vel_i + (gravity - bias_i) * dt;

            // Residuals are the difference between predicted and actual position and velocity at time j
            residuals[0] = predicted_pos_j.x() - pos_j.x();
            residuals[1] = predicted_pos_j.y() - pos_j.y();
            residuals[2] = predicted_pos_j.z() - pos_j.z();
            residuals[3] = predicted_vel_j.x() - vel_j.x();
            residuals[4] = predicted_vel_j.y() - vel_j.y();
            residuals[5] = predicted_vel_j.z() - vel_j.z();

            return true;
        }

    }
}