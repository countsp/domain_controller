# check for CAN1 message in terminal
```
sudo ip link set can1 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on
candump can1 &
```

# can2rostopic arouse imu & navsat ros2 topic
```
source can_reader/install/setup.bash
ros2 run can_reader can2ros_imu_nav_node
```
### output
imu and navsat message at both 100hz （can rate set to 100hz）
0x21B（GNSS纬度经度）,0x31B(GNSS高度)为10hz,0x20(纬度经度),0x30B（高度）为100hz

![Screenshot from 2024-03-14 16-08-58](https://github.com/countsp/domain_controller/assets/102967883/034cf239-a92f-4b71-b594-6213b7000921)

### 组合导航上DTU配置
#### 流程
DTU模块提供定位差分信号，组合导航MS61111接收差分信号进行解算，在GNSS-COM输出精确位置报文。

###### （optional）使用STRSVR软件可以代替DTU模块产生定位差分信号：
1.将GNSS-COM1与Windows通过rs232-usb连接

2.打开STRSVR如图设置Input Output

2. Input-Opt中配置千寻账号

3. Output-Opt中配置输出COM口

4.点击start，Input与Output灯闪烁

5.下方应该显示经纬度信息

6.打开SSCOM输入
```
log gpgga ontime 1
saveconfig
```

###### DTU配置
1.将DTU（COM1）与Windows通过rs232-usb连接

2.打开dtucfg软件

3.设置串口号

4.开始配置--上电

5.在右侧配置千寻账号，点击修改，点击修改参数

6.停止，下电

7.开始配置--上电，信息应该更新了

8.将DTU上TX/RX交换，与组合导航GNSS-COM1相连接

###### 时间同步

1.将GNSS-COM2与Windows通过rs232-usb连接

2.打开SSCOM输入
```
unlog gpgga() (为了清除之前gpgga的配置)
log gprmc ontime 1
saveconfig
config com2 9600
```
