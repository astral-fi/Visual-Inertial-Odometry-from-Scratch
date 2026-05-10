#pragma once
#include <string>
#include <optional>
#include "types.hpp"
#include <fstream>
#include <filesystem>

namespace vio{
    class DatasetReader{
    private:
        std::filesystem::path base_path;
        std::ifstream imu_file;
        std::ifstream image_file;
        std::optional<ImuMeasurement> next_imu_measurement;
        std::optional<ImageMeasurement> next_image_measurement;

        cv::Mat loadImage(const std::string& image_file_path);
        std::optional<ImuMeasurement> nextImuMeasurement();

        std::optional<ImageMeasurement> nextImageMeasurement();
    public:
        DatasetReader(const std::string &dataset_path);
        std::optional<SensorMeasurement> getNextMeasurement();

    };
}
