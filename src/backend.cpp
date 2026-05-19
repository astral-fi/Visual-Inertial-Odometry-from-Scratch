#include "vio/backend.hpp"

namespace vio{
    const Eigen::Vector3d GRAVITY(0, 0, -9.81);
    void Backend::process(const vio::TrackedFrame& tracked_frame, const vio::PreintegratedIMU& preintegrated_imu, const vio::ImuMeasurement& imu_measurement){
        if(this->backend_state == BackendState::UNINITIALIZED){ 
            init_imu_buffer.push_back(imu_measurement);
            if(init_imu_buffer.size() >= 200){
                current_state.timestamp = tracked_frame.timestamp;
                current_state.position = Eigen::Vector3d::Zero();
                current_state.velocity = Eigen::Vector3d::Zero();
                current_state.orientation = Eigen::Quaterniond::Identity();
                current_state.accel_bias = Eigen::Vector3d::Zero();
                current_state.gyro_bias = Eigen::Vector3d::Zero();
                backend_state = BackendState::INITIALIZING; // Transition to tracking state after initialization
            }
        } else if(this->backend_state == BackendState::INITIALIZING){
            Eigen::Vector3d avg_accel = Eigen::Vector3d::Zero();
            Eigen::Vector3d avg_gyro = Eigen::Vector3d::Zero();
            for(auto &imu : init_imu_buffer){
                avg_accel += imu.accel;
                avg_gyro += imu.gyro;
            }
            avg_accel /= init_imu_buffer.size();
            avg_accel.normalize();
            avg_gyro /= init_imu_buffer.size();
            current_state.accel_bias = avg_accel - GRAVITY;
            avg_accel.normalize();
            current_state.gyro_bias = avg_gyro;
            current_state.orientation = Eigen::Quaterniond::FromTwoVectors(avg_accel, Eigen::Vector3d(0, 0, 1));
            backend_state = BackendState::TRACKING; // Transition to tracking state after initialization
            init_imu_buffer.clear();
        } else if(this->backend_state == BackendState::TRACKING){
            // Update the current state using the preintegrated IMU measurements
            current_state.timestamp += preintegrated_imu.accumulated_time;
            current_state.position += current_state.velocity * preintegrated_imu.accumulated_time + 0.5*GRAVITY*preintegrated_imu.accumulated_time*preintegrated_imu.accumulated_time + preintegrated_imu.accumulated_rotation.toRotationMatrix() * preintegrated_imu.accumulated_position;
            current_state.velocity += GRAVITY * preintegrated_imu.accumulated_time + preintegrated_imu.accumulated_rotation.toRotationMatrix() * preintegrated_imu.accumulated_velocity;
            current_state.orientation = preintegrated_imu.accumulated_rotation * current_state.orientation;
            current_state.accel_bias = preintegrated_imu.accel_bias;
            current_state.gyro_bias = preintegrated_imu.gyro_bias;  
            current_state.orientation.normalize(); // Normalize the quaternion to prevent drift over time


            this->state_history.push_back(current_state);
            this->sliding_window.push_back(current_state);
            if(this->sliding_window.size() > sliding_window_size){
                this->sliding_window.pop_front();
            }
            this->tracked_frames_history.push_back(tracked_frame);

            for(size_t i = 0; i < tracked_frame.features.size(); ++i){
                if(landmark_map.find(tracked_frame.features[i].id) == landmark_map.end()){
                    Landmark landmark;
                    landmark.id = tracked_frame.features[i].id;
                    Landmark::Observation observation;
                    observation.frame_index = static_cast<int>(tracked_frames_history.size() - 1);
                    observation.image_point = tracked_frame.features[i].position;
                    landmark.observations.push_back(observation);
                    landmark_map[tracked_frame.features[i].id] = landmark;
                } else {
                    Landmark::Observation observation;
                    observation.frame_index = static_cast<int>(tracked_frames_history.size() - 1);
                    observation.image_point = tracked_frame.features[i].position;
                    landmark_map[tracked_frame.features[i].id].observations.push_back(observation);
                }
            }

            std::vector<Eigen::Vector3d> triangulated_points;

            // For each feature observed in the current frame, we can perform triangulation if it has been observed in multiple frames
            for(auto& landmark_pair : landmark_map){
                int feature_id = landmark_pair.first;
                const auto& landmark = landmark_pair.second; 

                if(landmark.observations.size() > 3){
                    std::vector<Eigen::Matrix3d> projection_matrices;
                    std::vector<cv::Point2f> image_points;
                    const auto& state = state_history.back();
                    const auto& tracked_frame = tracked_frames_history.back();
                    vio::TrackedFrame first_tracked_frame = tracked_frames_history[landmark.observations[0].frame_index];
                    vio::State first_state = state_history[landmark.observations[0].frame_index];

                    if((first_state.position - state.position).norm() < 0.1){
                        continue; // Skip triangulation if the camera has not moved significantly
                    }

                    Eigen::Matrix3d rotation_matrix = state.orientation.toRotationMatrix().transpose();
                    Eigen::Matrix3d first_rotation_matrix = first_state.orientation.toRotationMatrix().transpose(); 
                    Eigen::Vector3d current_ray = rotation_matrix*Eigen::Vector3d((landmark.observations[landmark.observations.size()-1].image_point - cv::Point2f(320, 240))).normalized(); // Assuming the principal point is at the center of the image
                    Eigen::Vector3d first_ray = first_rotation_matrix*Eigen::Vector3d(landmark.observations[0].image_point - cv::Point2f(320, 240)).normalized(); // Assuming the principal point is at the center of the image
                    double angle = std::acos(current_ray.dot(first_ray));
                    if(angle < 0.01){
                        continue; // Skip triangulation if the angle between the rays is too small
                    }


                    for(size_t idx : {landmark.observations[0].frame_index, landmark.observations[landmark.observations.size()-1].frame_index}){
                        const vio::State& state = state_history[idx]; 
                        const auto& tracked_frame = tracked_frames_history[idx]; // Get the latest tracked frame for this feature
                        const auto& feature = landmark.observations[idx].image_point; // Get the image point for this feature in the latest tracked frame
                        Eigen::Matrix3d rotation_matrix = state.orientation.toRotationMatrix().transpose();
                        Eigen::Vector3d translation_vector = -rotation_matrix * state.position; // Assuming the camera is at the origin of the IMU frame, we can use the position to get the translation
                        Eigen::Matrix<double, 3, 4> projection_matrix;
                        projection_matrix.block<3,3>(0,0) = rotation_matrix;
                        projection_matrix.block<3,1>(0,3) = translation_vector;
                        projection_matrices.push_back(projection_matrix);
                        image_points.push_back(feature);
                    }

                    Eigen::Vector3d triangulated_point = triangulatePoints(projection_matrices, image_points);
                    if(triangulated_point.z() > 0){
                        triangulated_points.push_back(triangulated_point);
                    }
                    else{
                        continue;
                    }
                    landmark_map[feature_id].is_triangulated = true;
                    std::copy(triangulated_point.data(), triangulated_point.data() + 3, landmark_map[feature_id].position);
                    
                }
            }
        }
    }

    Eigen::Vector3d Backend::triangulatePoints(const std::vector<Eigen::Matrix3d>& projection_matrices, const std::vector<cv::Point2f>& image_points){

        Eigen::MatrixXd A(2 * image_points.size(), 4);
        for (size_t i = 0; i < image_points.size(); ++i) {
            const auto& P = projection_matrices[i];
            const auto& x = image_points[i].x;
            const auto& y = image_points[i].y;

            A.row(2 * i)     = x * P.row(2) - P.row(0);
            A.row(2 * i + 1) = y * P.row(2) - P.row(1);
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinV);
        Eigen::Vector4d homogeneous_point = svd.matrixV().col(3);

        return homogeneous_point.head<3>() / homogeneous_point[3];
    }
}