#include <vio/datareader.hpp>
#include <sstream>
#include <algorithm>

namespace vio{

    DatasetReader::DatasetReader(const std::string &dataset_path){
        this->base_path = std::filesystem::path(dataset_path);
        imu_file.open(base_path / "imu0" / "data.csv");
        image_file.open(base_path / "cam0" / "data.csv");
        if(!imu_file.is_open() || !image_file.is_open()){
            throw std::runtime_error("Failed to open dataset files.");
        }
        std::string line;
        std::getline(imu_file, line); // Skip header
        std::getline(image_file, line); // Skip header
        next_image_measurement = nextImageMeasurement();
        next_imu_measurement = nextImuMeasurement();
    }

    cv::Mat DatasetReader::loadImage(const std::string& image_file_path){
        std::filesystem::path image_path = base_path / "cam0" / "data" / image_file_path;
        cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
        if(image.empty()){
            throw std::runtime_error("Failed to load image: " + image_path.string());
        }
        return image;
    }

    std::optional<ImuMeasurement> DatasetReader::nextImuMeasurement(){
        std::string line;
        if(std::getline(imu_file, line)){
            std::istringstream ss(line);
            std::string token;
            ImuMeasurement measurement;

            std::getline(ss, token, ',');
            measurement.timestamp = std::stoll(token);

            std::getline(ss, token, ',');
            measurement.accel.x() = std::stod(token);
            std::getline(ss, token, ',');
            measurement.accel.y() = std::stod(token);
            std::getline(ss, token, ',');
            measurement.accel.z() = std::stod(token);

            std::getline(ss, token, ',');
            measurement.gyro.x() = std::stod(token);
            std::getline(ss, token, ',');
            measurement.gyro.y() = std::stod(token);
            std::getline(ss, token);
            measurement.gyro.z() = std::stod(token);

            return measurement;
        } else {
            return std::nullopt; // No more IMU measurements
        }
    }

    std::optional<ImageMeasurement> DatasetReader::nextImageMeasurement(){
        std::string line;
        if(std::getline(image_file, line)){
            std::istringstream ss(line);
            std::string token;
            ImageMeasurement measurement;

            std::getline(ss, token, ',');
            measurement.timestamp = std::stoll(token);

            std::getline(ss, token);
            // Add this right after you parse image_file_path
            token.erase(std::remove(token.begin(), token.end(), '\r'), token.end());
            token.erase(std::remove(token.begin(), token.end(), '\n'), token.end());
            measurement.image = loadImage(token);

            return measurement;
        } else {
            return std::nullopt; // No more image measurements
        }
    }


    std::optional<SensorMeasurement> DatasetReader::getNextMeasurement(){
        if(!next_imu_measurement.has_value()){
            next_imu_measurement = nextImuMeasurement();
        }
        if(!next_image_measurement.has_value()){
            next_image_measurement = nextImageMeasurement();
        }

        if(next_imu_measurement.has_value() && next_image_measurement.has_value()){
            if(next_imu_measurement->timestamp < next_image_measurement->timestamp){
                SensorMeasurement measurement = *next_imu_measurement;
                next_imu_measurement.reset();
                return measurement;
            } else {
                SensorMeasurement measurement = *next_image_measurement;
                next_image_measurement.reset();
                return measurement;
            }
        } else if(next_imu_measurement.has_value()){
            SensorMeasurement measurement = *next_imu_measurement;
            next_imu_measurement.reset();
            return measurement;
        } else if(next_image_measurement.has_value()){
            SensorMeasurement measurement = *next_image_measurement;
            next_image_measurement.reset();
            return measurement;
        } else {
            return std::nullopt; // No more measurements
        }
    }


}