#include "gps_parser/gphpd_parser.h"

void GPHPDParser::enqueue(const std::string& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    dataQueue_ = data;
    cv_.notify_one();
}

bool GPHPDParser::parseImpl(const std::string& data) {
    // 1. 基本校验
    if (data.empty() || data.find("$GPHPD") != 0) {
        std::cerr << "Invalid GPHPD data format" << std::endl;
        return false;
    }
 
    // 2. 校验和验证
    if (!validateChecksum(data)) {
        std::cerr << "GPHPD checksum validation failed" << std::endl;
        return false;
    }
 
    // 3. 分割数据字段 
    std::vector<std::string> fields = split(data.substr(0, data.find('*')), ',');
 
    // 4. 字段数量检查 
    if (fields.size() < 16) {
        std::cerr << "Incomplete GPHPD data fields size = " << fields.size() << std::endl;
        return false;
    }
 
    try {
        // 5. 解析各字段
        currentData_.gpsWeek = std::stoi(fields[1]);
        currentData_.gpsTime = std::stod(fields[2]);
        currentData_.heading = std::stod(fields[3]);
        currentData_.pitch = std::stod(fields[4]);
        currentData_.track = std::stod(fields[5]);
        currentData_.latitude = std::stod(fields[6]);
        currentData_.longitude = std::stod(fields[7]);
        currentData_.altitude = std::stod(fields[8]);
        currentData_.ve = std::stod(fields[9]);
        currentData_.vn = std::stod(fields[10]);
        currentData_.vu = std::stod(fields[11]);
        currentData_.baseline = std::stod(fields[12]);
        currentData_.nsv1 = std::stoi(fields[13]);
        currentData_.nsv2 = std::stoi(fields[14]);
        
        // 解析状态字段 
        if (fields[15].length() >= 2) {
            currentData_.status = std::stoi(fields[15], nullptr, 16);
        } else {
            currentData_.status = 0;
        }
    } catch (const std::exception& e) {
        std::cerr << "GPHPD data parsing error: " << e.what() << std::endl;
        return false;
    }
    return true;
}
 
void GPHPDParser::printResult() const {
    const auto& d = currentData_;
    
    std::cout << "====== GPHPD Data @ " << d.gpsWeek << " week, " << d.gpsTime << " seconds ======" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    
    // 姿态信息
    std::cout << "Attitude: Heading=" << d.heading << "°, Pitch=" << d.pitch 
              << "°, Track=" << d.track << "°" << std::endl;
    
    // 位置信息 
    std::cout << "Position: Lat=" << d.latitude << ", Lon=" << d.longitude 
              << ", Alt=" << d.altitude << "m" << std::endl;
    
    // 速度信息 
    std::cout << "Velocity: Ve=" << d.ve << "m/s, Vn=" << d.vn 
              << "m/s, Vu=" << d.vu << "m/s" << std::endl;
    
    // 其他信息 
    std::cout << "Baseline: " << d.baseline << "m, SV1=" << d.nsv1 
              << ", SV2=" << d.nsv2 << std::endl;
    
    // 状态信息解析 
    int lowNibble = d.status;
    
    const char* states[] = {
        "Initializing", "", "", "GPS positioning",
        "GPS orientation", "RTK positioning", "", "",
        "", "", "", "RTK orientation",
        "", "", "", ""
    };
    
    std::cout << "Status: " << states[lowNibble] << std::endl;
    std::cout << "========================================" << std::endl;
}
void GPHPDParser::pubResult(const ros::Publisher &publisher) {
    const auto& d = currentData_;
    sensor_msgs::NavSatFixPtr nav_msg(new sensor_msgs::NavSatFix());
    nav_msg->header.frame_id = "global";
    nav_msg->header.stamp = ros::Time::now();
    nav_msg->latitude = d.latitude;                                // rad
    nav_msg->longitude = d.longitude;                               // rad
    nav_msg->altitude = d.altitude;                                // rad

    sensor_msgs::NavSatStatus nav_status_msg;
    nav_status_msg.status = d.status < 5 ? 0:2; // RTK status
    nav_status_msg.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    nav_msg->status = nav_status_msg;
    publisher.publish(nav_msg);
}

void GPHPDParser::start() {
    if (!running_) {
        running_ = true;
        workerThread_ = std::thread(&GPHPDParser::workerThread, this);
    }
}
 
void GPHPDParser::stop() {
    if (running_) {
        running_ = false;
        cv_.notify_all();
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
    }
}
 
void GPHPDParser::workerThread() {
    while (ros::ok() && running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return !dataQueue_.empty() || !running_; });
        
        if (!running_) break;
        
        std::string data = std::move(dataQueue_);
        dataQueue_.clear();
        lock.unlock();
        
        if(parseImpl(data))
        {
            std::lock_guard<std::mutex> output_lock(g_outputMutex);
            printResult();
            pubResult(publisher_);
        }
    }
}
