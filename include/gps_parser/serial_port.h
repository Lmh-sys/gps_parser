#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H
 
#include <string>
#include <memory>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <serial/serial.h>
#include <iostream>
#include "ros/ros.h"
#define LOG_DEBUG() \
    std::cout << "[" << __DATE__ << " " << __TIME__ << "] " \
              << "function:" << __FUNCTION__ \
              << " (File:" << __FILE__ << ":" << __LINE__ << ")" << std::endl 
class SerialPort {
public:
    // 数据回调类型（收到完整数据行时触发） 
    using DataCallback = std::function<void(const std::string&)>;
 
    /**
     * @brief 构造函数 
     * @param port 串口设备路径（如"/dev/gnss0"）
     * @param baudrate 波特率（默认115200）
     * @param data_bits 数据位（5-8，默认8）
     * @param parity 校验位（N/E/O，默认N）
     * @param stop_bits 停止位（1/2，默认1）
     */
    explicit SerialPort(const std::string& port, 
                       int baudrate = 115200);
    ~SerialPort();
 
    // 禁止拷贝和赋值 
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
 
    void stop() {
        running_ = false;
        if (read_thread_.joinable()) read_thread_.join();
        if (ser_ptr_ && ser_ptr_->isOpen()) ser_ptr_->close();
    }
    void start(){
      if (!running_ && callback_ && ser_ptr_->isOpen()) {
          running_ = true;
          read_thread_ = std::thread(&SerialPort::readThread, this);
      }
    }
    
    /// @brief 设置数据回调函数
    void setDataCallback(DataCallback callback);
 
private:
    void connect();
    std::shared_ptr<serial::Serial> ser_ptr_;
 
    // 线程控制 
    void readThread();
    std::atomic<bool> running_{false};
    std::thread read_thread_;
    DataCallback callback_;
    std::mutex callback_mutex_;
    std::string port_;
    int baudrate_;
};
 
#endif // SERIAL_PORT_H 