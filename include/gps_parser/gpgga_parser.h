#ifndef GPGGA_PARSER_H 
#define GPGGA_PARSER_H
 
#include "parser_base.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

// GNSS定位定向消息集
struct GPGGAData {
    std::string utcTime;
    double latitude, longitude, altitude;
    int status;
    int nsv1;
    double hdop, msl, altref;
    double diffAge, diffStation;
};
 
class GPGGAParser : public ParserBase {
public:
    GPGGAParser(ros::Publisher publisher):publisher_(publisher){};
    ~GPGGAParser() override = default;
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
    void pubResult(const ros::Publisher &publisher) override;
private:
    GPGGAData currentData_;
    // 数据队列
    std::string dataQueue_;
    ros::Publisher publisher_;
};
 
#endif // GPGGA_PARSER_H 