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
able to see imu and navsat message at both 100hz （can rate set to 100hz）
0x21B（GNSS纬度经度）,0x31B(GNSS高度)为10hz,0x20(纬度经度),0x30B（高度）为100hz
![Screenshot from 2024-03-14 16-05-55](https://github.com/countsp/domain_controller/assets/102967883/e7f3b008-fdb7-4595-9015-eb738f51e48f)
