#include "gps_parser/gpgga_parser.h"

void GPGGAParser::enqueue(const std::string& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    dataQueue_ = data;
    cv_.notify_one();
}

bool GPGGAParser::parseImpl(const std::string& data) {
    // 1. 基本校验
    if (data.empty() || data.find("$GPGGA") != 0) {
        std::cerr << "Invalid GPGGA data format" << std::endl;
        return false;
    }
 
    // 2. 校验和验证
    if (!validateChecksum(data)) {
        std::cerr << "GPGGA checksum validation failed" << std::endl;
        return false;
    }
 
    // 3. 分割数据字段 
    std::vector<std::string> fields = split(data.substr(0, data.find('*')), ',');
    // 4. 字段数量检查 
    if (fields.size() < 15) {
        std::cerr << "Incomplete GPGGA data fields size = " << fields.size() << std::endl;
        std::cerr << "fields = ";
        for (const auto& str : fields) { 
            std::cout << str << ", ";
        }
        std::cout << std::endl;
        return false;
    }
 
    try {
        // 5. 解析各字段
        currentData_.utcTime = fields[1];
        double lat = std::stod(fields[2]);
        int deg = static_cast<int>(lat/100);
        currentData_.latitude = deg + (lat - deg*100)/60.0;
        if(fields[3] == "S") currentData_.latitude *= -1;
        double lon = std::stod(fields[4]);
        deg = static_cast<int>(lon/100);
        currentData_.longitude = deg + (lon - deg*100)/60.0;
        if(fields[5] == "W") currentData_.longitude *= -1;
        currentData_.status = fields[6].empty() ? 0 : std::stoi(fields[6]);
        currentData_.nsv1 = fields[7].empty() ? 0 : std::stoi(fields[7]);
        currentData_.hdop = fields[8].empty() ? 99.9 : std::stod(fields[8]);
        currentData_.msl = fields[9].empty() ? 0.0 : std::stod(fields[9]);
        currentData_.altref = fields[11].empty() ? 0.0 : std::stod(fields[11]);
        currentData_.diffAge = fields[13].empty() ? 0.0 : std::stod(fields[13]);
        currentData_.diffStation = fields[14].empty() ? 0 : std::stoi(fields[14]);
    } catch (const std::exception& e) {
        std::cerr << "GPGGA data parsing error: " << e.what() << std::endl;
        return false;
    }
    return true;
}
 
void GPGGAParser::printResult() const {
    const auto& d = currentData_;
    
    // 状态描述映射
    static const char* STATUS_MAP[] = {
        "Initializing", "Single Point", "DGPS", "Reserved",
        "Fixed RTK", "Float RTK", "Estimating", "Manual",
        "Simulation", "WAAS"
    };
    
    std::cout << "\n====== GPGGA Data @ " << d.utcTime << " seconds ======" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Position: " << std::abs(d.latitude) << (d.latitude>=0?"°N ":"°S ")
              << std::abs(d.longitude) << (d.longitude>=0?"°E":"°W") << "\n"
              << "Altitude: " << d.msl << "m (MSL) | " << d.altref << "m (Geoid)\n"
              << "Status: " << STATUS_MAP[d.status%10] 
              << " | Satellites: " << d.nsv1 << " | HDOP: " << d.hdop << "\n";
    
    if(d.diffAge > 0) {
        std::cout << "DGPS: Age=" << d.diffAge << "s Station=" << d.diffStation << "\n";
    }
    std::cout << "=======================\n";
    std::cout << std::resetiosflags(std::ios_base::floatfield);
}

void GPGGAParser::pubResult(const ros::Publisher &publisher) {
    const auto& d = currentData_;
    sensor_msgs::NavSatFixPtr nav_msg(new sensor_msgs::NavSatFix());
    nav_msg->header.frame_id = "global";
    nav_msg->header.stamp = ros::Time::now();
    nav_msg->latitude = d.latitude;                                // rad
    nav_msg->longitude = d.longitude;                               // rad
    nav_msg->altitude = d.altitude;                                // rad
    nav_msg->position_covariance[0] = d.hdop * d.hdop; // m*m
    nav_msg->position_covariance[4] = d.hdop * d.hdop; // m*m
    nav_msg->position_covariance[8] = d.hdop * d.hdop; // m*m
    sensor_msgs::NavSatStatus nav_status_msg;
    nav_status_msg.status = d.status != 4 ? 0:2; // RTK status
    nav_status_msg.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    nav_msg->status = nav_status_msg;
    publisher.publish(nav_msg);
}

void GPGGAParser::start() {
    if (!running_) {
        running_ = true;
        workerThread_ = std::thread(&GPGGAParser::workerThread, this);
    }
}
 
void GPGGAParser::stop() {
    if (running_) {
        running_ = false;
        cv_.notify_all();
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
    }
}
 
void GPGGAParser::workerThread() {
    while (ros::ok() && running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this]() { return !dataQueue_.empty(); });
        
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
