#ifndef GPRMC_PARSER_H 
#define GPRMC_PARSER_H
 
#include "parser_base.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
// 导航推荐定位信息
struct GPRMCData {
    std::string utcTime;         // hhmmss.ss 
    char status;            // A/V
    double latitude, longitude;
    double speed;            // 节(knots)
    double course;           // 真北航向(度)
    std::string utcTime1;         // ddmmyy
    double magneticVariation;// 磁偏角(度)
    char modeIndicator;     // A/D/E/N 
};

class GPRMCParser : public ParserBase {
public:
    GPRMCParser(ros::Publisher publisher):publisher_(publisher){};
    ~GPRMCParser() override = default;
    // 线程函数 
    void workerThread();
    // 启动解析线程
    void start();
    // 停止解析线程 
    void stop();
    void enqueue(const std::string& data) override;
 
protected:
    bool parseImpl(const std::string& data) override;
    void printResult() const override;
private:
    GPRMCData currentData_;
    // 数据队列
    std::string dataQueue_;
    ros::Publisher publisher_;
};
 
#endif // GPRMC_PARSER_H 