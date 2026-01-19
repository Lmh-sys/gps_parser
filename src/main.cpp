#include "gps_parser/gpfpd_parser.h"
#include "gps_parser/gphpd_parser.h"
#include "gps_parser/gtimu_parser.h"
#include "gps_parser/gpgga_parser.h"
#include "gps_parser/gprmc_parser.h"
#include "gps_parser/serial_port.h"
#include <memory>
#include <vector>
#include <unordered_map>
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "gps_parser");
    ros::NodeHandlePtr node_ptr(new ros::NodeHandle());
    ros::Publisher imu_publisher_ = node_ptr->advertise<sensor_msgs::Imu>("imu", 100, true);
    ros::Publisher gnss_gpfpd_publisher_ = node_ptr->advertise<sensor_msgs::NavSatFix>("gnss_gpfpd", 100, true);
    ros::Publisher gnss_gpgga_publisher_ = node_ptr->advertise<sensor_msgs::NavSatFix>("gnss_gpgga", 10, true);
    ros::Publisher gnss_gprmc_publisher_ = node_ptr->advertise<sensor_msgs::NavSatFix>("gnss_gprmc", 10, true);
    ros::Publisher gnss_gphpd_publisher_ = node_ptr->advertise<sensor_msgs::NavSatFix>("gnss_gphpd", 10, true);
    
    // 创建解析器实例
    std::unordered_map<std::string, std::shared_ptr<ParserBase>> parser_map = {
        {"$GTIMU", std::make_shared<GTIMUParser>(imu_publisher_)}, 
        {"$GPFPD", std::make_shared<GPFPDParser>(gnss_gpfpd_publisher_)},
        {"$GPGGA", std::make_shared<GPGGAParser>(gnss_gpgga_publisher_)},
        // {"$GPRMC", std::make_shared<GPRMCParser>(gnss_gprmc_publisher_)},
        {"$GPHPD", std::make_shared<GPHPDParser>(gnss_gphpd_publisher_)},
    };
 
    // 启动所有解析器 
    for (auto& [key, parser] : parser_map) {
        parser->start(); 
    }
    // std::string str = "$GPFPD,2401,532628.810,4.133,2.578,-0.712,22.57913897,113.93795461,33.09,0.579,0.270,-0.119,0.157,8,14,4B*00";
    // parser_map["$GPFPD"]->enqueue(str);
    // std::string str = "$GPGGA,031747.40,2234.7605,N,11356.2719,E,4,23,0.8,36.42,M,-3.50,M,00,2189*46";
    // parser_map["$GPGGA"]->enqueue(str);

    auto port = std::make_unique<SerialPort>("/dev/gnss0", 115200);
    port->setDataCallback([&parser_map](const std::string& data) {
        // 提取消息头（前6字节）
        constexpr size_t HEADER_LEN = 6;
        if (data.length() < HEADER_LEN) return;
        
        std::string header = data.substr(0, HEADER_LEN);
        
        // 哈希查找对应解析器 
        auto it = parser_map.find(header);
        if (it != parser_map.end()) {
            // std::cout << "\n data:" << data << std::endl;
            it->second->enqueue(data);
        } else {
            // std::cerr << "[WARN] Unknown protocol: " << header << std::endl;
        }
    });
    
    // 安全退出
    port->stop();
    
    // 停止所有解析器
    for (auto& [key, parser] : parser_map) {
        parser->stop(); 
    }
    
    return 0;
}