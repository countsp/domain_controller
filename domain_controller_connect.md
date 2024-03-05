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

**接线**：

ch128x1与cb64s1通过四合一车载以太网接入域控上LAN-1接口(在域控以太网接口中ip设置为192.168.1.102)

##### 在win上位机中配置设备ip
**接192.168.1.102：**

ch128x1设备：ip：192.168.1.200  msop_port: 2368  difop_port: 2369

ch64s1_1（左侧）设备：ip：192.168.1.201  msop_port: 2370  difop_port: 2371

ch64s1_2 (右侧) 设备：ip：192.168.1.202  msop_port: 2372  difop_port: 2373

**接192.168.1.103：**

通过usb以太网接入域控usb(在域控以太网接口中ip设置为192.168.1.103) 重新买千兆转换器

c32W 设备ip:192.168.1.203 msop_port: 2374  difop_port: 2375

**启动操作**:
```
# for cb64s1_1 and cb64s1_2 and ch128

# source工作目录
source ~/lslidar128_ws/install/setup.bash

# 启动launch节点方法
ros2 launch lslidar_driver lslidar_ch128x1_ch64w_double_launch.py 
```

```
# for c32w lidar（自启动rviz2）

# source工作目录
source ~/lsc32w/install/setup.bash

# 启动launch节点方法
ros2 launch lslidar_driver lslidar_c32_launch.py

```
![Screenshot from 2024-03-05 15-50-43](https://github.com/countsp/domain_controller/assets/102967883/9634d29c-c45e-4a67-af73-4474247aae87)
##### requirements
c32的yaml文件中设置c32_type: c32_70     # c32_32: 垂直角度是的30度c32   c32_70: 垂直角度是的70度c32(c32w)  c32_90: 垂直角度是的90度c32(ch32w)次2

ping通192.168.102与192.168.1.103（200,201,202可能ping不通）

网络设置：

![Screenshot from 2024-03-05 15-20-54](https://github.com/countsp/domain_controller/assets/102967883/2383d2ca-192c-44c4-aa33-76b67c919cc0)

ETH2为LAN-1 3*激光雷达口，ip为192.168.102 ，在 Wired connetion 3 中设置
![Screenshot from 2024-03-05 15-26-08](https://github.com/countsp/domain_controller/assets/102967883/2e5809e5-a6ff-49c0-bfbc-cc114d721aa7)

TP-Link为C32 USB以太网口 ，ip为192.168.103， 在TP-Link Ethernet中设置
![Screenshot from 2024-03-05 15-25-05](https://github.com/countsp/domain_controller/assets/102967883/daab9565-d1a3-4124-bcf3-d9062299e234)


