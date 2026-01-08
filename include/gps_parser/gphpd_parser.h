#ifndef GPHPD_PARSER_H 
#define GPHPD_PARSER_H
 
#include "parser_base.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
// GNSS定位定向消息集
struct GPHPDData {
    int gpsWeek;
    double gpsTime;
    double heading, pitch, track;
    double latitude, longitude, altitude;
    double ve, vn, vu;
    double baseline;
    int nsv1, nsv2;
    int status;
};
 
class GPHPDParser : public ParserBase {
public:
    GPHPDParser(ros::Publisher publisher):publisher_(publisher){};
    ~GPHPDParser() override = default;
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
    GPHPDData currentData_;
    // 数据队列
    std::string dataQueue_;
    ros::Publisher publisher_;
};
 
#endif // GPHPD_PARSER_H 