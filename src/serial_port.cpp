#include "gps_parser/serial_port.h"

 
//---------------------------
// SerialPort成员函数实现 
//---------------------------
 
SerialPort::SerialPort(const std::string& port, int baudrate) {
  try 
  {
    ser_ptr_ = std::make_shared<serial::Serial>();
    ser_ptr_->setPort(port); 
    ser_ptr_->setBaudrate(baudrate); 
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); //超时定义，单位：ms
    ser_ptr_->setTimeout(to); 
    ser_ptr_->open(); 
  } 
  catch (serial::IOException& e) 
  { 
    std::cout << "Fail to open serial port\n";
  }

  ser_ptr_->flushInput();//在开始正式接收数据前先清除串口的接收缓冲区 	
}
 
SerialPort::~SerialPort() {
    stop();
}

void SerialPort::setDataCallback(DataCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    callback_ = std::move(callback);
    start();
}

void SerialPort::readThread() {
    constexpr size_t BUFFER_SIZE = 1024;
    uint8_t buffer[BUFFER_SIZE];
    std::string residual; // 存储不完整的数据片段 
    running_ = true;
    while (ros::ok() && running_ && ser_ptr_->isOpen()) {
        int numinbuf = ser_ptr_->available();//available()返回从串口缓冲区读回的字符数
        try {
            // 读取原始数据
            if( numinbuf > 0){
              size_t bytes_read = ser_ptr_->read(buffer, numinbuf);
              if (bytes_read == numinbuf) {
                  residual.append(reinterpret_cast<char*>(buffer), bytes_read);
                  // 按行分割处理（支持\r\n和\n两种结尾）
                  size_t pos = 0;
                  while ((pos = residual.find('\n')) != std::string::npos) {
                      std::string line = residual.substr(0, pos);
                      residual.erase(0, pos + 1);
  
                      // 线程安全调用回调
                      if (!line.empty()) {
                          std::lock_guard<std::mutex> cb_lock(callback_mutex_);
                          if (callback_) callback_(line);
                      }
                  }
              }
            }
            
        } catch (const serial::IOException& e) {
            std::cerr << "[SerialPort] Read error: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}
