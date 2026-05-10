#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "vio/datareader.hpp"
#include "vio/frontend.hpp"

int main(){
    vio::DatasetReader reader("/home/astral-fi/vicon_room1/V1_01_easy/mav0");
	int count = 0;
	vio::Frontend frontend(true);
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
			frontend.processImage(image_measurement);
		}
    }
    return 0;
}
