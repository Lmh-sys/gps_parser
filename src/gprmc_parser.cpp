#include "gps_parser/gprmc_parser.h"

void GPRMCParser::enqueue(const std::string& data) {
    std::lock_guard<std::mutex> lock(mutex_);
    dataQueue_ = data;
    cv_.notify_one();
}

bool GPRMCParser::parseImpl(const std::string& data) {
    // 1. 基本校验
    if (data.empty() || data.find("$GPRMC") != 0) {
        std::cerr << "Invalid GPRMC data format" << std::endl;
        return false;
    }
 
    // 2. 校验和验证
    if (!validateChecksum(data)) {
        std::cerr << "GPRMC checksum validation failed" << std::endl;
        return false;
    }
 
    // 3. 分割数据字段 
    std::vector<std::string> fields = split(data.substr(0, data.find('*')), ',');
 
    // 4. 字段数量检查 
    if (fields.size() < 13) {
        std::cerr << "Incomplete GPRMC data fields size= "  << fields.size() << std::endl;
        std::cerr << "fields = ";
        for (const auto& str : fields) { 
            std::cout << str << ", ";
        }
        std::cout << std::endl;
        return false;
    }
    // for (size_t i = 0; i < fields.size(); ++i) {
    //     std::cout << i << ":" << fields[i] << std::endl;
    // }

    try {
        // 5. 解析各字段
        // UTC时间 (hhmmss.ss)
        currentData_.utcTime = fields[0];
 
        // 定位状态 (A/V)
        currentData_.status = fields[2][0];
 
        // 纬度解析 (ddmm.mmmm → 十进制度)
        double lat = std::stod(fields[3]);
        int deg = static_cast<int>(lat / 100);
        currentData_.latitude = deg + (lat - deg * 100) / 60.0;
        if(fields[4] == "S") currentData_.latitude *= -1;
 
        // 经度解析 (dddmm.mmmm → 十进制度)  
        double lon = std::stod(fields[5]);
        deg = static_cast<int>(lon / 100);
        currentData_.longitude = deg + (lon - deg * 100) / 60.0;
        if (fields[6] == "W") {
            currentData_.longitude *= -1;
        }
 
        // 运动参数
        currentData_.speed = fields[7].empty() ? 0.0f : std::stof(fields[6]);  // 节(knots)
        currentData_.course = fields[8].empty() ? 0.0f : std::stof(fields[7]); // 航向(度)
 
        // UTC日期 (ddmmyy)
        currentData_.utcTime1 = fields[9];
 
        // 磁偏角
        currentData_.magneticVariation = fields[10].empty() ? 0.0f : std::stof(fields[9]);
        if (fields[11] == "W") {
            currentData_.magneticVariation *= -1;
        }
 
        // 模式指示 (A/D/E/N)
        currentData_.modeIndicator = fields[12][0];
    } catch (const std::exception& e) {
        std::cerr << "GPRMC data parsing error: " << e.what() << std::endl;
        return false;
    }
    return true;
}
 
void GPRMCParser::printResult() const {
    const auto& d = currentData_;
    
    // 状态描述映射
    const std::unordered_map<char, std::string> GPGA_MODE_MAP = {
        {'A', "Autonomous (Standalone)"},
        {'D', "DGPS Fix"},
        {'E', "Estimated (Dead Reckoning)"},
        {'N', "No Fix"}
    };
    const std::unordered_map<char, std::string> FIX_STATUS_MAP = {
        {'A', "Active Fix"},
        {'V', "No Fix"}
    }; 

    std::cout << "\n====== GPRMC Data @ " << d.utcTime << " seconds ======" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Position: " << std::abs(d.latitude) << (d.latitude>=0?"°N ":"°S ")
              << std::abs(d.longitude) << (d.longitude>=0?"°E":"°W") << "\n"
              << "Status: " << d.status << "Mode: " << d.modeIndicator << "\n";
    std::cout << "Status: " << FIX_STATUS_MAP.at(d.status) << "Mode: " << GPGA_MODE_MAP.at(d.modeIndicator) << "\n";
}


void GPRMCParser::start() {
    if (!running_) {
        running_ = true;
        workerThread_ = std::thread(&GPRMCParser::workerThread, this);
    }
}
 
void GPRMCParser::stop() {
    if (running_) {
        running_ = false;
        cv_.notify_all();
        if (workerThread_.joinable()) {
            workerThread_.join();
        }
    }
}
 
void GPRMCParser::workerThread() {
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
            // printResult();
            pubResult(publisher_);
        }
    }
}
