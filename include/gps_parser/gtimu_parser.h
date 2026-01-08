#ifndef GTIMU_PARSER_H 
#define GTIMU_PARSER_H
#include "sensor_msgs/Imu.h"
#include "parser_base.h"

struct GTIMUData {
    int gpsWeek;
    double gpsTime;
    double gyro_x, gyro_y, gyro_z;
    double acc_x, acc_y, acc_z;
    double temperature;
};
 
class GTIMUParser : public ParserBase {
public:
    GTIMUParser(ros::Publisher publisher):publisher_(publisher){};
    ~GTIMUParser() override = default;
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
    GTIMUData currentData_;
    // 数据队列
    std::string dataQueue_;
    ros::Publisher publisher_;
};
 
#endif // GTIMU_PARSER_H 