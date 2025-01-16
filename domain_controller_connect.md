#### 激光雷达车载以太网配置
Ubuntu settings-Ethernet-IPV4
设置为manual，Address为192.168.1.102，掩码为 255.255.255.0（102为LAN1，LAN1上设备都与102连接）

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
**要求：**

在域控系统内能够以rostopic的格式看到摄像头信号

**编译命令(以CH128X1为例)：**
```
cd ~/lslidar128_ws
colcon build
```

**接线**：

CH128x1与CB64S1通过四合一车载以太网接入域控上LAN-1接口(在域控以太网接口中ip设置为192.168.1.102)

C32W通过usb以太网接入域控usb(在域控以太网接口中ip设置为192.168.1.103) 

**配置** 

在win上位机中配置设备ip

**接192.168.1.102（LAN1车载无线网）：**

CH128X1设备：ip：192.168.1.200  msop_port: 2368  difop_port: 2369

CB64S1_1（左侧）  设备：ip：192.168.1.201  msop_port: 2370  difop_port: 2371

CB64S1_2 (右侧)   设备：ip：192.168.1.202  msop_port: 2372  difop_port: 2373

**接192.168.1.103（USB以太网）：**

C32W    设备ip:192.168.1.203    msop_port: 2374    difop_port: 2375

**lidar启动操作**:
```
# for cb64s1_1 and cb64s1_2 and ch128x1

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
c32的yaml文件中设置c32_type: c32_70     # c32_32: 垂直角度是的30度c32   c32_70: 垂直角度是的70度c32(c32w)  c32_90: 垂直角度是的90度c32(ch32w)

ping通192.168.102与192.168.1.103（200,201,202可能ping不通，不影响）

网络设置：

![Screenshot from 2024-03-05 15-20-54](https://github.com/countsp/domain_controller/assets/102967883/2383d2ca-192c-44c4-aa33-76b67c919cc0)

ETH2为LAN-1 3*激光雷达口，ip为192.168.102 ，在 Wired connetion 3 中设置
![Screenshot from 2024-03-05 15-26-08](https://github.com/countsp/domain_controller/assets/102967883/2e5809e5-a6ff-49c0-bfbc-cc114d721aa7)

TP-Link为C32 USB以太网口 ，ip为192.168.103， 在TP-Link Ethernet中设置
![Screenshot from 2024-03-05 15-25-05](https://github.com/countsp/domain_controller/assets/102967883/daab9565-d1a3-4124-bcf3-d9062299e234)


# 组合导航配置

1.组合导航 com1（配置输入口） 通过 rs232-usb 链接到电脑

2.右上角配置端口，配置后应该显示已经连接。

3.可以点击参数配置，点击更新，被更新处应该变红。

4.(无需RTK)在走直线或者8字后，Inspva组合导航状态会变为3。

5.(RTK) GNSS定位标志为1时，是单点解。4/5时，说明RTK已经接入。



# RTK配置
### 组合导航上RTK配置（在DTU失效时，使用能上网的笔记本代替）

#### 流程


###### （optional）使用STRSVR软件可以代替DTU模块产生定位差分信号：

1.将GNSS-COM1与Windows通过rs232-usb连接（电脑输出差分信号给GNSS-COM1）

2.打开STRSVR如图，设置Input和Output的Opt

![image](https://github.com/countsp/domain_controller/assets/102967883/5eaa5d9c-7f98-4795-ae21-22735d70c4cf)

2. Input-Opt中配置千寻账号，Mountpoint可以AUTO

![inputoption](https://github.com/countsp/domain_controller/assets/102967883/0d9e52b7-12bb-494f-8b2e-d6d516edb7f1)


3. Output-Opt中只需配置输出COM口

![image](https://github.com/countsp/domain_controller/assets/102967883/b4f94c2a-0695-4922-b293-4e09bae20b4a)

4.点击start，Input与Output灯闪烁绿光，下方应该显示经纬度信息

![Uploading 屏幕截图 2025-01-16 102452.png…]()


5. 如果上面灯是黄色，重新点开Input->Opt，设置Mountpoint，下拉选一个，如果下拉菜单为空，点击Get Mountup再回到主界面点击Start，等待一会点击Close，再重新操作这一步。

 
6.（不再需要）打开SSCOM输入，固化设置
```
log gpgga ontime 1
saveconfig
```

### DTU模块提供定位差分信号，组合导航MS-61111接收差分信号进行解算，在GNSS-COM1输出精确位置报文。
###### DTU配置
1.将DTU（COM1）与Windows通过rs232-usb连接

2.打开dtucfg软件

3.设置串口号

4.开始配置--上电

5.在右侧配置千寻账号，点击修改，点击修改参数

6.停止，下电

7.开始配置--上电，信息应该更新了

8.将DTU上TX/RX交换，与组合导航GNSS-COM1相连接

---

# 时间同步（MS-6111组合导航与镭神C32w/CH128X1/CB64S20进行PPS硬同步，以C32w为例）

1.将GNSS-COM2与Windows通过rs232-usb连接

2.打开SSCOM输入
```
unlog gpgga() (为了清除之前gpgga的配置)
log gprmc ontime 1
saveconfig
config com2 9600（设置9600波特率）
```


3.将GNSS-COM2（RS232）的TX引出，作为GPRMC数据线，连接到C32w的GPS_Rec线上（P4的pin3）

4.PPS线从组合导航引出，连接到C32w的GPS_PPS线上（P4的pin2）

5.GND线从从组合导航引出，连接到C32w的GPS_GND线上（P4的pin5）

![image](https://github.com/countsp/domain_controller/assets/102967883/49f7d5ed-4e55-4e3f-afd3-1543a7e39553)

6.以太网连接电脑，配置以太网ip（192.168.1.203），在上位机上设置msop_port: 2374 difop_port: 2375。

在Param中点击Parameter，可见PPS Status变为Connect。（Enable。PPS相位统一设置，例如均为180）

32线配置
![c320327](https://github.com/countsp/domain_controller/assets/102967883/426cadcb-a529-4759-b341-6de59566c4ee)
64线配置
![64](https://github.com/countsp/domain_controller/assets/102967883/87f5e6a6-a3d4-4861-8f0b-1cb781201f26)
```
注意，CH128X1的datasheet有误，pin6中（椭圆头），1号接PPS，5号接地，6号接GPRMC
```
