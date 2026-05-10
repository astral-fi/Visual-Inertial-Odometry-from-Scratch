#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "types.hpp"

namespace vio{
    class Frontend{
        private:
            cv::Mat prev_image;
            std::vector<cv::Point2f> prev_keypoints = {};
            std::vector<cv::Point2f> keypoints = {};
            std::vector<int> keypoint_ids = {};
            int next_keypoint_id = 0;
            bool show_gui = true;
        public:
            Frontend(bool show_gui);
            void processImage(const ImageMeasurement& image_measurement);
    };
}