#ifndef PARSER_BASE_H 
#define PARSER_BASE_H 
 
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <iostream>
#include <sstream>
#include <iomanip>
#include "ros/ros.h"
#include <unordered_map>

const double GRAVITY = 9.805;
#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

class ParserBase {
public:
    ParserBase();
    virtual ~ParserBase();
 
    // 禁止拷贝和赋值
    ParserBase(const ParserBase&) = delete;
    ParserBase& operator=(const ParserBase&) = delete;
    // 线程函数 
    virtual void workerThread() = 0;
    // 启动解析线程
    virtual void start() = 0;
    // 停止解析线程 
    virtual void stop() = 0;
    virtual void enqueue(const std::string& data) = 0;

protected:
    // 子类必须实现的纯虚函数
    virtual bool parseImpl(const std::string& data) = 0;
    virtual void printResult() const = 0;
    virtual void pubResult(const ros::Publisher &publisher){};

    std::vector<std::string> split(const std::string &s, char delimiter=',');
    // 辅助函数：校验和验证 
    bool validateChecksum(const std::string& data);
 
    // GPS领先UTC的秒数
    const int GPS_LEAP_SECOND = 18;
    
    // Unix时间戳转GPS时间
    std::tuple<int, double> unix_second_to_gps_time(double seconds) {
        double second_gps = seconds + GPS_LEAP_SECOND - 315964800;
        int week = static_cast<int>(std::floor(second_gps / 604800));
        double sow = second_gps - week * 604800;
        return std::make_tuple(week, sow);
    }
    
    // GPS时间转Unix时间戳
    double gps_time_to_unix_second(int week, double sow) {
        return sow + week * 604800 + 315964800 - GPS_LEAP_SECOND;
    }
    ros::Time gpggaTimeToROSTime(const std::string& utc_time_str) {
        // 1. 解析时间 
        double hh = std::stod(utc_time_str.substr(0, 2));
        double mm = std::stod(utc_time_str.substr(2, 2));
        double ss = std::stod(utc_time_str.substr(4));
    
        // 2. 获取当前日期 
        std::time_t now = std::time(nullptr);
        std::tm tm_utc = *std::gmtime(&now);
        tm_utc.tm_hour = static_cast<int>(hh);
        tm_utc.tm_min = static_cast<int>(mm);
        tm_utc.tm_sec = static_cast<int>(ss);
    
        // 3. 转换为ROS时间
        std::time_t time_utc = std::mktime(&tm_utc) - timezone;
        return ros::Time(
            time_utc,
            static_cast<uint32_t>((ss - tm_utc.tm_sec) * 1e9)
        );
    }
    // 线程控制 
    std::atomic<bool> running_;
    std::thread workerThread_;
    
    // 线程同步
    std::mutex mutex_;
    std::condition_variable cv_;
    std::mutex g_outputMutex;
};
 
#endif // PARSER_BASE_H 