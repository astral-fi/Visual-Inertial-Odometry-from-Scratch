#include "types.hpp"

namespace vio{
    enum class BackendState{
        UNINITIALIZED,
        INITIALIZING,
        TRACKING
    };
    class Backend{
        private:
            std::vector<ImuMeasurement> init_imu_buffer;
            std::vector<vio::State> state_history;
            std::vector<vio::TrackedFrame> tracked_frames_history;
            std::deque<vio::State> sliding_window;
            const size_t sliding_window_size = 10;
            struct Landmark{
                double position[3];
                int id;
                bool is_triangulated;
                struct Observation{
                    int frame_index;
                    cv::Point2f image_point;
                };
                std::vector<Observation> observations;
                Landmark(){
                    position[0] = 0;
                    position[1] = 0;
                    position[2] = 0;
                    id = -1;
                    is_triangulated = false;
                }
            };
            std::map<int, Landmark> landmark_map; // Map from feature ID to Landmark
        public:
            vio::BackendState backend_state = BackendState::UNINITIALIZED;
            vio::State current_state;
            Backend() = default;
            void process(const vio::TrackedFrame& tracked_frame, const vio::PreintegratedIMU& preintegrated_imu, const vio::ImuMeasurement& imu_measurement);
            Eigen::Vector3d triangulatePoints(const std::vector<Eigen::Matrix3d>& projection_matrices, const std::vector<cv::Point2f>& image_points);
    };
}