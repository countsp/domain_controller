#### 激光雷达车载以太网配置
Ubuntu settings-Ethernet-IPV4
设置为manual，Address为192.168.1.102，掩码为 255.255.255.0（102作为LAN1，LAN1上设备都与102连接）
![Screenshot from 2024-03-04 19-31-12](https://github.com/countsp/domain_controller/assets/102967883/e9d61be8-f651-495a-a6a9-a16f9b953a29)

ifconfig后看到eth1
![Screenshot from 2024-03-04 21-23-47](https://github.com/countsp/domain_controller/assets/102967883/73a4290c-624e-4b2c-a73f-51b7dd04fcf7)

## 域控连接
### 1.摄像头
**要求**：在域控系统内能够以rostopic的格式看到摄像头信号，格式为sensor_msg::msg::Image

**操作**: 
```
ctrl-alt-t
ros2 topic list    #需能看到摄像头rostopic
ros2 topic echo /${rostopic_name}   # 需能输出原始数字信号
ros2 topic hz /${rostopic_name}   # 输出频率需与摄像头匹配
ros2 topic info /${rostopic_name} # 输出格式应该为sensor_msg::msg::Image

```
**接线**：通过四合一Fakra接入域控上GMSL-1接口

**启动操作**（需要供应商补充）：写入bashrc中，参考格式如下
```
# 启动设备
# for ch128 lidar

# source工作目录
source ~/lslidar128_ws/install/setup.bash

# 启动launch节点方法（以备注形式标注）
# ros2 launch lslidar_driver lslidar_ch128x1_launch.py  
```

---

### 2.激光雷达
要求：在域控系统内能够以rostopic的格式看到摄像头信号
操作：
```
cd ~/lslidar128_ws
colcon build
```

**接线**：通过四合一车载以太网接入域控上LAN-1接口(接口ip设置为192.168.1.102)


**启动操作**:
```
# 启动设备
# for ch128 lidar

# source工作目录
source ~/lslidar128_ws/install/setup.bash

# 启动launch节点方法（以备注形式标注）
# ros2 launch lslidar_driver lslidar_ch128x1_launch.py  
```
