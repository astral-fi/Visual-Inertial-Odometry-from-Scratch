#include <vio/frontend.hpp>
#include <opencv2/imgproc.hpp>


namespace vio{
    Frontend::Frontend(const vio::Camera& camera, bool show_gui){
        this->camera = camera;
        this->show_gui = show_gui;
    }
    vio::TrackedFrame Frontend::processImage(const ImageMeasurement& image_measurement){
        cv::Mat image = image_measurement.image;
        std::vector<cv::Point2f> corners;
        if(prev_image.empty()){
            cv::goodFeaturesToTrack(image, corners, 300, 0.01, 30);
            prev_image = image.clone();
            prev_keypoints = corners;
            for(const auto& pt : corners){
                keypoint_ids.push_back(next_keypoint_id++);
            }
            std::vector<cv::Point2f> normalized_keypoints = camera.undistortPoints(corners);
            std::vector<vio::TrackedFeature> tracked_features;
            for(int i = 0; i < normalized_keypoints.size(); ++i){
                vio::TrackedFeature feature;
                feature.id = keypoint_ids[i];
                feature.position = corners[i];
                feature.normalized_position = normalized_keypoints[i];
                tracked_features.push_back(feature);
            }
            vio::TrackedFrame tracked_frame;
            tracked_frame.timestamp = image_measurement.timestamp;
            tracked_frame.features = tracked_features;
            return tracked_frame;
        }
        else{
            std::vector<cv::Point2f> tracked_keypoints;
            std::vector<uchar> status;
            std::vector<float> err;
            cv::calcOpticalFlowPyrLK(prev_image, image, prev_keypoints, tracked_keypoints, status, err);
            std::vector<cv::Point2f> valid_tracked_keypoints;
            std::vector<cv::Point2f> valid_old_keypoints;
            std::vector<int> valid_keypoint_ids;
            for(size_t i = 0; i < status.size(); ++i){
                if(status[i]){
                    valid_tracked_keypoints.push_back(tracked_keypoints[i]);
                    valid_old_keypoints.push_back(prev_keypoints[i]);
                    valid_keypoint_ids.push_back(keypoint_ids[i]);
                }
            }   
            prev_image = image.clone();
            prev_keypoints = valid_tracked_keypoints;
            keypoint_ids = valid_keypoint_ids;
            if(show_gui){
                cv::Mat display_image;
                cv::cvtColor(image, display_image, cv::COLOR_GRAY2BGR);
                for(int i = 0; i < prev_keypoints.size(); ++i){
                    cv::circle(display_image, valid_tracked_keypoints[i], 5, cv::Scalar(0, 255, 0), -1);
                    cv::line(display_image, valid_old_keypoints[i], valid_tracked_keypoints[i], cv::Scalar(255, 0, 0), 2);
                }
                cv::imshow("Keypoints", display_image);
                cv::waitKey(1); // Display the image for a short time
            }

            if(valid_tracked_keypoints.size() < 100){
                cv::Mat mask(image.size(), CV_8UC1, cv::Scalar(255));
                for(const auto& pt : valid_tracked_keypoints){
                    cv::circle(mask, pt, 15, cv::Scalar(255), -1);
                }
                cv::goodFeaturesToTrack(image, corners, 300 - valid_tracked_keypoints.size(), 0.01, 30, mask);
                prev_keypoints.insert(prev_keypoints.end(), corners.begin(), corners.end());
                for(size_t i = 0; i < corners.size(); ++i){
                    keypoint_ids.push_back(next_keypoint_id++);
                }
            }
            std::vector<cv::Point2f> normalized_keypoints = camera.undistortPoints(valid_tracked_keypoints);
            std::vector<vio::TrackedFeature> tracked_features;
            for(int i = 0; i < normalized_keypoints.size(); ++i){
                vio::TrackedFeature feature;
                feature.id = valid_keypoint_ids[i];
                feature.position = valid_tracked_keypoints[i];
                feature.normalized_position = normalized_keypoints[i];
                tracked_features.push_back(feature);
            }
            vio::TrackedFrame tracked_frame;
            tracked_frame.timestamp = image_measurement.timestamp;
            tracked_frame.features = tracked_features;
            return tracked_frame;

        }
    }

}