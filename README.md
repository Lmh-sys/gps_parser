## GPS驱动使用示例
### 说明
本驱动根据北京星网宇达科技股份有限公司的Newton-M2组合导航设备输出的数据协议进行解析。
目前仅实现了"$GTIMU","$GPFPD","$GPGGA",$GPHPD"等四种消息格式的解析。

### 依赖
```shell
sudo apt-get install ros-$ROS_DISTRO-serial
# 串口号设置、查询对应串口id填入99-usb-serial.rules的ATTRS{idVendor}和ATTRS{idProduct}
# KERNEL选择串口可能的串口号、SYMLINK为重定义的串口名字
lsusb
sudo gedit /etc/udev/rules.d/99-usb-serial.rules
KERNEL=="ttyACM[0-9]*", ATTRS{idVendor}=="5953", ATTRS{idProduct}=="5543", MODE:="0777", SYMLINK+="gnss0"
sudo udevadm control --reload-rules
sudo udevadm trigger
```
### 原理
根据GPS厂商提供的GPS输出格式解析出对应的GPS消息发布

```yaml
$GPFPD,2401,532628.810,4.133,2.578,-0.712,22.57913897,113.93795461,33.09,0.579,0.270,-0.119,0.157,8,14,4B*00"
$GPGGA,031747.40,2234.7605,N,11356.2719,E,4,23,0.8,36.42,M,-3.50,M,00,2189*46
```
例如，如上消息格式解析后对应的消息如下
```yaml
====== GPGGA Data @ 031747.40 seconds ======
Position: 22.579342°N Attitude: Heading=113.937865°E
Altitude: 36.420000m (MSL) | -3.500000m (Geoid)
Status: Fixed RTK | Satellites: 23 | HDOP: 0.800000
=======================
====== GPFPD Data @ 2401 week, 532629 seconds ======
4.133000°, Pitch=2.578°, Roll=-0.712°
Position: Lat=22.5791, Lon=113.938, Alt=33.09m
Velocity: Ve=0.579m/s, Vn=0.27m/s, Vu=-0.119m/s
Baseline: 0.157m, SV1=8, SV2=14
Status: RTK orientation (RTK fixed mode)
========================================
```
### 运行步骤
```shell
roslaunch gps_parser gps_parser.launch
````
输出话题
imu、gnss_gpfpd、gnss_gpgga、gnss_gphpd

### 参考
[指令手册](doc/%E6%95%B0%E6%8D%AE%E6%89%8B%E5%86%8C/Newton%E6%8C%87%E4%BB%A4%E6%89%8B%E5%86%8C.doc) 
[说明书](doc/%E6%95%B0%E6%8D%AE%E6%89%8B%E5%86%8C/Newton-M2%E4%BD%BF%E7%94%A8%E7%BB%B4%E6%8A%A4%E8%AF%B4%E6%98%8E%E4%B9%A6.pdf)