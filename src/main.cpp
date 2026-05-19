#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "vio/datareader.hpp"
#include "vio/frontend.hpp"
#include "vio/IMUPreintegration.hpp"
#include "vio/backend.hpp"


int main(){
    vio::DatasetReader reader("/home/astral-fi/vicon_room1/V1_01_easy/mav0");
	vio::Camera camera("/home/astral-fi/vicon_room1/V1_01_easy/mav0/cam0/sensor.yaml");
	int count = 0;
	vio::Frontend frontend(camera,true);
	vio::IMUPreintegration imu_preintegration;
	vio::Backend backend;
    while(auto measurement = reader.getNextMeasurement()){
        // Process the measurement
		if(std::holds_alternative<vio::ImuMeasurement>(*measurement) ){
			auto imu_measurement = std::get<vio::ImuMeasurement>(*measurement);
			imu_preintegration.integrate(imu_measurement);

		} else if(std::holds_alternative<vio::ImageMeasurement>(*measurement)){
			auto image_measurement = std::get<vio::ImageMeasurement>(*measurement);
			vio::TrackedFrame tracked_frame = frontend.processImage(image_measurement);
			vio::PreintegratedIMU preintegrated_imu = imu_preintegration.getPreintegratedIMU();
			imu_preintegration.reset(backend.current_state.gyro_bias, backend.current_state.accel_bias);
		}

		if(backend.backend_state == vio::BackendState::UNINITIALIZED && std::holds_alternative<vio::ImuMeasurement>(*measurement))
		{
			auto imu_measurement = std::get<vio::ImuMeasurement>(*measurement);
			backend.process(vio::TrackedFrame{}, vio::PreintegratedIMU{}, imu_measurement);
		}
		else if(backend.backend_state == vio::BackendState::INITIALIZING && std::holds_alternative<vio::ImuMeasurement>(*measurement)){
			auto imu_measurement = std::get<vio::ImuMeasurement>(*measurement);
			backend.process(vio::TrackedFrame{}, vio::PreintegratedIMU{}, imu_measurement);
		}
		else if(backend.backend_state == vio::BackendState::TRACKING && std::holds_alternative<vio::ImageMeasurement>(*measurement)){
			auto image_measurement = std::get<vio::ImageMeasurement>(*measurement);
			vio::TrackedFrame tracked_frame = frontend.processImage(image_measurement);
			vio::PreintegratedIMU preintegrated_imu = imu_preintegration.getPreintegratedIMU();
			imu_preintegration.reset(backend.current_state.gyro_bias, backend.current_state.accel_bias);
			backend.process(tracked_frame, preintegrated_imu, vio::ImuMeasurement{});
		
		}
    }
    return 0;
}
