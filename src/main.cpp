#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "vio/datareader.hpp"

int main(){
    vio::DatasetReader reader("/home/astral-fi/vicon_room1/V1_01_easy/mav0");
	int count = 0;
    while(auto measurement = reader.getNextMeasurement()){
        // Process the measurement
		if(std::holds_alternative<vio::ImuMeasurement>(*measurement)){
			auto imu_measurement = std::get<vio::ImuMeasurement>(*measurement);
			count++;
			if(count % 100 == 0){
				std::cout << "100 IMU Messages" << std::endl;
			}
		} else if(std::holds_alternative<vio::ImageMeasurement>(*measurement)){
			auto image_measurement = std::get<vio::ImageMeasurement>(*measurement);
			// Process Image measurement
			std::cout << "Image Measurement at timestamp: " << image_measurement.timestamp << std::endl;
			cv::imshow("Image", image_measurement.image);
			cv::waitKey(1); // Display the image for a short time
		}
    }
    return 0;
}
