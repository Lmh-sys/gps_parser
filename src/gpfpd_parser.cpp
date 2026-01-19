#include "gps_parser/gpfpd_parser.h"

void GPFPDParser::enqueue(const std::string& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    dataQueue_ = data;
    cv_.notify_one();
}

bool GPFPDParser::parseImpl(const std::string& data) {
    // 1. 基本校验
    if (data.empty() || data.find("$GPFPD") != 0) {
        std::cerr << "Invalid GPFPD data format" << std::endl;
        return false;
    }
 
    // 2. 校验和验证
    if (!validateChecksum(data)) {
        std::cerr << "GPFPD checksum validation failed:\n" << data << std::endl;
        return false;
    }
 
    // 3. 分割数据字段 
    std::vector<std::string> fields = split(data.substr(0, data.find('*')), ',');
 
    // 4. 字段数量检查 
    if (fields.size() < 16) {
        std::cerr << "Incomplete GPFPD data fields size = " << fields.size() << std::endl;
        return false;
    }
 
    try {
        // 5. 解析各字段
        currentData_.gpsWeek = std::stoi(fields[1]);
        currentData_.gpsTime = std::stod(fields[2]);
        currentData_.heading = std::stod(fields[3]);
        currentData_.pitch = std::stod(fields[4]);
        currentData_.roll = std::stod(fields[5]);
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
        std::cerr << "GPFPD data parsing error: " << e.what() << std::endl;
        return false;
    }
    return true;
}
 
void GPFPDParser::printResult() const {
    const auto& d = currentData_;
    
    std::cout << "====== GPFPD Data @ " << d.gpsWeek << " week, " << d.gpsTime << " seconds ======" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    
    // 姿态信息
    std::cout << "Attitude: Heading=" << d.heading << "°, Pitch=" << d.pitch 
              << "°, Roll=" << d.roll << "°" << std::endl;
    
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
    int lowNibble = d.status & 0x0F;
    int highNibble = (d.status >> 4) & 0x0F;
    
    const char* states[] = {
        "Initializing", "Coarse align", "Fine align", "GPS positioning",
        "GPS orientation", "RTK positioning", "DMI combined", "DMI calibration",
        "Pure inertial", "Zero-speed corr", "VG mode", "RTK orientation",
        "Nav init", "", "", "INS error"
    };
    
    const char* modes[] = {
        "GPS", "BD", "Dual-mode", "",
        "RTK fixed", "RTK float", "", "",
        "", "", "", "",
        "", "", "", ""
    };
    
    std::cout << "Status: " << states[lowNibble] 
              << " (" << modes[highNibble] << " mode)" << std::endl;
    std::cout << "========================================" << std::endl;
}

void GPFPDParser::pubResult(const ros::Publisher &publisher) {
    const auto& d = currentData_;
    sensor_msgs::NavSatFixPtr nav_msg(new sensor_msgs::NavSatFix());
    nav_msg->header.frame_id = "global";
    nav_msg->header.stamp = ros::Time::now();
    nav_msg->latitude = d.latitude;                                
    nav_msg->longitude = d.longitude;                              
    nav_msg->altitude = d.altitude;                                

    sensor_msgs::NavSatStatus nav_status_msg;
    switch (d.status & 0x0F)
    {
    case 0:
      nav_status_msg.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
      break;
    case 3:
      nav_status_msg.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      break;
    case 4:
      nav_status_msg.status = sensor_msgs::NavSatStatus::STATUS_FIX;
      break;
    case 5:
      nav_status_msg.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
      break;  
    case 11:
      nav_status_msg.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
      break;  
    default:
      break;
    }
    nav_status_msg.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    nav_msg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    nav_msg->position_covariance[0] = d.nsv1; // 从天线卫星数量
    nav_msg->position_covariance[1] = d.nsv2; // 主天线卫星数量
    nav_msg->position_covariance[2] = d.baseline; // 基线
    nav_msg->position_covariance[3] = d.ve; // 东向速度
    nav_msg->position_covariance[4] = d.vn; // 北向速度
    nav_msg->position_covariance[5] = d.vu; // 天向速度
    nav_msg->position_covariance[6] = d.heading; // 航向
    nav_msg->position_covariance[7] = d.pitch; // 姿态
    nav_msg->position_covariance[8] = d.roll; // 姿态
    nav_msg->status = nav_status_msg;
    publisher.publish(nav_msg);
}

void GPFPDParser::start() {
    if (!running_) {
        running_ = true;
        workerThread_ = std::thread(&GPFPDParser::workerThread, this);
    }
}
 
void GPFPDParser::stop() {
    if (running_) {
        running_ = false;
        cv_.notify_all();
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
    }
}
 
void GPFPDParser::workerThread() {
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
