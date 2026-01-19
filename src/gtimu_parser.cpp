#include "gps_parser/gtimu_parser.h"
#include <iostream>
#include <vector>
#include <sstream>
#include <iomanip>
 

#include <iomanip>
#include <cmath>
 
bool GTIMUParser::parseImpl(const std::string& data) {
    // 1. 基础校验
    if (data.empty() || data.find("$GTIMU") != 0) {
        std::cerr << "[GTIMU] Invalid header" << std::endl;
        return false;
    }
 
    // 2. 校验和验证（复用基类方法）
    if (!validateChecksum(data)) {
        std::cerr << "[GTIMU] Checksum failed" << std::endl;
        return false;
    }
 
    // 3. 分割有效字段（忽略*后的校验部分）
    auto fields = split(data.substr(0, data.find('*')), ',');
    if (fields.size() < 10) {
        std::cerr << "[GTIMU] Missing fields size= " << fields.size() << std::endl;
        return false;
    }
 
    try {
        // 4. 核心字段解析 
        currentData_.gpsWeek = std::stoi(fields[1]);
        currentData_.gpsTime = std::stod(fields[2]);
        currentData_.gyro_x = std::stod(fields[3]);
        currentData_.gyro_y = std::stod(fields[4]);
        currentData_.gyro_z = std::stod(fields[5]);
        currentData_.acc_x = std::stod(fields[6]);
        currentData_.acc_y = std::stod(fields[7]);
        currentData_.acc_z = std::stod(fields[8]);
        currentData_.temperature = std::stod(fields[9]);
    } 
    catch (const std::exception& e) {
        std::cerr << "[GTIMU] Parse error: " << e.what() << std::endl;
        return false;
    }
    return true;
}
 
void GTIMUParser::printResult() const {
    const auto& d = currentData_;
    
    // 1. 时间信息（GPS周+秒转UTC时间）
    std::cout << "=== GTIMU Data @ Week " << d.gpsWeek 
              << " (" << std::fixed << std::setprecision(3) << d.gpsTime << "s) ===\n";
    
    // 2. 运动参数（科学计数法）
    std::cout << std::scientific << std::setprecision(4);
    std::cout << "Gyro: X=" << d.gyro_x << " Y=" << d.gyro_y << " Z=" << d.gyro_z << "\n";
    
    // 3. 加速度参数（固定小数）
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Acc:  X=" << d.acc_x << " Y=" << d.acc_y << " Z=" << d.acc_z << "\n";
    
    // 4. 环境参数 
    std::cout << "Temp: " << std::setprecision(1) << d.temperature << "°C\n";
    std::cout << "=======================\n";
}
void GTIMUParser::pubResult(const ros::Publisher &publisher) {
    sensor_msgs::Imu imu;
    const auto& d = currentData_;
    
    // imu.header.stamp = ros::Time(gps_time_to_unix_second(d.gpsWeek, d.gpsTime));
    imu.header.stamp = ros::Time::now();
    imu.header.frame_id = "imu";
    imu.header.seq = d.temperature;   //温度
    
    // set orientation:
    imu.orientation.w = 1;
    imu.orientation.x = 0;
    imu.orientation.y = 0;
    imu.orientation.z = 0;

    // set angular velocity:
    imu.angular_velocity.x = -d.gyro_x * DEG2RAD;
    imu.angular_velocity.y = -d.gyro_y * DEG2RAD;
    imu.angular_velocity.z = d.gyro_z * DEG2RAD;

    // set linear acceleration:
    imu.linear_acceleration.x = -d.acc_x * GRAVITY;
    imu.linear_acceleration.y = -d.acc_y * GRAVITY;
    imu.linear_acceleration.z = d.acc_z * GRAVITY;

    publisher.publish(imu);
}


void GTIMUParser::start() {
    if (!running_) {
        running_ = true;
        workerThread_ = std::thread(&GTIMUParser::workerThread, this);
    }
}
 
void GTIMUParser::stop() {
    if (running_) {
        running_ = false;
        cv_.notify_all();
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
    }
}
 
void GTIMUParser::workerThread() {
    while (ros::ok() && running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return !dataQueue_.empty(); });
        
        std::string data = std::move(dataQueue_);
        dataQueue_.clear();
        lock.unlock();
        
        if(parseImpl(data))
        {
            std::lock_guard<std::mutex> output_lock(g_outputMutex);
            // printResult();
            pubResult(publisher_);
        }
    }
}
void GTIMUParser::enqueue(const std::string& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    dataQueue_ = data;
    cv_.notify_one();
}