[ref](https://blog.csdn.net/zardforever123/article/details/132538871)

# 参数配置

（1）修改 autoware.launch.xml 文件
在src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml第17行将launch_sensing_driver中的true改成false，不使用Autoware自带的驱动程序，可以直接复制以下替换第17行。

```
<arg name="launch_sensing_driver" default="false" description="launch sensing driver"/>
```


（2）修改 gnss.launch.xml 文件
在src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_launch/launch/gnss.launch.xml第4行将coordinate_system中的1改成2，可以直接复制以下替换第4行。

```
<arg name="coordinate_system" default="2" description="0:UTM, 1:MGRS, 2:PLANE"/>
```

在第5行添加如下：

```
<arg name="plane_zone" default="0"/>
```

（3）如果不使用GNSS，则在/src/universe/autoware.universe/launch/tier4_localization_launch/launch/util/util.launch.xml将GNSS设置为不可用

```
<arg name="gnss_enabled" value="false"/>
<!-- <arg name="gnss_enabled" value="true"/> -->
```


（4）预处理参数修改
可能需要修改点云预处理的参数，可以在下面找到autoware_universe/src/universe/autoware.universe/launch/tier4_localization_launch/config，尤其是体素滤波参数：
在这里插入图片描述

![image](https://github.com/user-attachments/assets/7a012cd3-e773-454f-b058-db2774c79626)


---

# 传感器数据接入
![image](https://github.com/user-attachments/assets/14f0ae01-2f1f-4120-968e-9d2322686947)


**/control/command/contral_cmd** 为Autoware发出的用于控制小车底盘的消息，/op_ros2_agent会将其转化为CAN消息，进而控制底盘；

**/sensing/imu/tamagawa/imu_raw** 为IMU消息，用于获取里程计，frame_id为tamagawa/imu_link；

**/sensing/gnss/ublox/nav_sat_fix**为GNSS消息，用于位姿初始化（可选），frame_id为gnss_link；

**/sensing/camera/traffic_light/image_raw** 为图像消息，用于识别红绿灯，frame_id为camera4/camera_link；

**/vehicla/status** 为车反馈的速度等状态消息（具体可以看CAN代码）；

**/Carla_pointcloud** 为Carla输出的点云，Autoware输入的点云为 /sensing/lidar/top/outlier_filtered/pointcloud和/sensing/lidar/concatenated/pointcloud（frame_id均为base_link），下面的/carla_pointcloud_interface节点会将/Carla_pointcloud转化， /sensing/lidar/top/outlier_filtered/pointcloud用于定位，/sensing/lidar/concatenated/pointcloud用于感知；

## Lidar

![image](https://github.com/user-attachments/assets/33dd8e5d-8734-4629-aec6-9989229fdbb4)

如下节点，将/points_raw（frame_id为velodyne）（或者是你的雷达驱动输出）坐标转换到base_link（需要雷达外参），然后发布 /sensing/lidar/top/outlier_filtered/pointcloud和/sensing/lidar/concatenated/pointcloud

```
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class LidarTransformNode : public rclcpp::Node
{
public:
    LidarTransformNode() : Node("points_raw_transform_node")
    {
        // Declare parameters
        transform_x = this->declare_parameter("transform_x", 0.0);
        transform_y = this->declare_parameter("transform_y", 0.0);
        transform_z = this->declare_parameter("transform_z", 0.0);
        transform_roll = this->declare_parameter("transform_roll", 0.0);
        transform_pitch = this->declare_parameter("transform_pitch", 0.0);
        transform_yaw = this->declare_parameter("transform_yaw", 0.0);
        RadiusOutlierFilter = this->declare_parameter("radius_outlier_filter", 0.1);

        std::cout << "velodyne to base_link:" << std::endl
                  << "  transform_x:    " << transform_x << std::endl
                  << "  transform_y:    " << transform_y << std::endl
                  << "  transform_z:    " << transform_z << std::endl
                  << "  transform_roll: " << transform_roll << std::endl
                  << "  transform_pitch:" << transform_pitch << std::endl
                  << "  transform_yaw:  " << transform_yaw << std::endl
                  << "  RadiusOutlierFilter:  " << RadiusOutlierFilter << std::endl;

        // Initialize transformation parameters
        transform_stamp.transform.translation.x = transform_x;
        transform_stamp.transform.translation.y = transform_y;
        transform_stamp.transform.translation.z = transform_z;
       
        tf2::Quaternion quaternion;
        quaternion.setRPY(transform_roll, transform_pitch, transform_yaw);
        transform_stamp.transform.rotation.x = quaternion.x();
        transform_stamp.transform.rotation.y = quaternion.y();
        transform_stamp.transform.rotation.z = quaternion.z();
        transform_stamp.transform.rotation.w = quaternion.w();

        // Create publishers
        publisher_1 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/top/outlier_filtered/pointcloud", 10);
        publisher_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/concatenated/pointcloud", 10);

        // Subscribe to raw point cloud messages
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/c32/lslidar_point_cloud", 10,
            std::bind(&LidarTransformNode::pointCloudCallback, this, std::placeholders::_1));
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Filter out points close to the sensor
        pcl::PointCloud<pcl::PointXYZI>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *xyz_cloud);
        for (size_t i = 0; i < xyz_cloud->points.size(); ++i)
        {
            if (sqrt(xyz_cloud->points[i].x * xyz_cloud->points[i].x + xyz_cloud->points[i].y * xyz_cloud->points[i].y +
                     xyz_cloud->points[i].z * xyz_cloud->points[i].z) >= RadiusOutlierFilter && !std::isnan(xyz_cloud->points[i].z))
            {
                pcl_output->points.push_back(xyz_cloud->points.at(i));
            }
        }
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*pcl_output, output);
        output.header = msg->header;

        // Perform coordinate transformation
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        pcl_ros::transformPointCloud("base_link", transform_stamp, output, transformed_cloud);

        // Publish the transformed point cloud messages
        publisher_1->publish(transformed_cloud);
        publisher_2->publish(transformed_cloud);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_1, publisher_2;

    double transform_x, transform_y, transform_z, transform_roll, transform_pitch, transform_yaw;
    double RadiusOutlierFilter;
    geometry_msgs::msg::TransformStamped transform_stamp;
};

int main(int argc, char **argv)
{
    // Initialize the node
    rclcpp::init(argc, argv);

    // Create an instance
    auto node = std::make_shared<LidarTransformNode>();

    // Spin the node
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

```

![Screenshot from 2024-07-18 10-58-59](https://github.com/user-attachments/assets/1a64e05d-19ce-4cdc-85d2-4b77f179bfca)


---

## imu
修改发布话题名称为/sensing/imu/tamagawa/imu_raw以及坐标系为tamagawa/imu_link

![image](https://github.com/user-attachments/assets/ab54d4a4-f473-4327-921a-c526ecd8afde)

![image](https://github.com/user-attachments/assets/1f164687-db38-4d82-8cb4-b4c9824139a2)

# 外参修改

安装几个使用的传感器，然后修改其外参（主要是修改IMU和用于识别红绿灯的相机，雷达我们已经在转换节点将其转换到base_link坐标系），外参需要标定（也可以直接拿尺子测量，但不准确），

标定方法参考：SLAM各传感器的标定总结：[Camera/IMU/LiDAR/](https://blog.csdn.net/zardforever123/article/details/130030767?spm=1001.2014.3001.5501)

src/sensor_kit/sample_sensor_kit_launch/sample_sensor_kit_description/config/sensor_kit_calibration.yaml：

# 车身参数

为了适配我们的车辆，需要修改车身参数（下面这些参数可以自己测量一下你的车子）src/vehicle/sample_vehicle_launch/sample_vehicle_description/config/vehicle_info.param.yaml和/universe/autoware.universe/vehicle/vehicle_info_util/config/vehicle_info.param.yaml两处：

# 统一启动

<launch>
    <!-- 雷达驱动 -->
    <include file="$(find-pkg-share rslidar_sdk)/launch/start.py" />
    <!-- 转换为Autoware需要的 -->
    <include file="$(find-pkg-share points_raw_transform)/launch/points_raw_transform.launch.xml" >
      <!-- LiDAR to base link -->
      <arg name="transform_x" value="0.75"/>
      <arg name="transform_y" value="0.35"/>
      <arg name="transform_z" value="0.6"/>
      <arg name="transform_roll" value="0.0"/>
      <arg name="transform_pitch" value="0.0"/>
      <arg name="transform_yaw" value="0.0"/>
      <arg name="RadiusOutlierFilter" value="2.5"/>
    </include>

    <!-- 相机驱动 -->
    <include file="$(find-pkg-share usb_cam)/launch/JR_HF868.launch.py" />

    <!-- 生成假的GNSS和IMU数据，用于调试 -->
    <!-- <include file="$(find-pkg-share gnss_imu_sim)/launch/both.launch.py" /> -->

    <!-- GNSS模块驱动(可选，这里未使用) -->
    <!-- <include file="$(find-pkg-share gnss_imu_sim)/launch/M620.launch.py" /> -->
    <!-- IMU模块驱动 -->
    <include file="$(find-pkg-share wit_ros2_imu)/rviz_and_imu.launch.py" />

    <!-- CAN驱动 -->
    <include file="$(find-pkg-share can_ros2_bridge)/can.launch.py" />

    <!-- 启动Autoware -->
    <include file="$(find-pkg-share autoware_launch)/launch/autoware.launch.xml">
      <!-- 地图路径 -->
      <arg name="map_path" value="/home/car/Autoware/QinHang"/>
      <!-- 车辆和传感器模型 -->
      <arg name="vehicle_model" value="sample_vehicle"/>
      <arg name="sensor_model" value="sample_sensor_kit"/>
      <!-- 不使用仿真时间 -->
      <arg name="use_sim_time" value="false"/>
      <!-- 不使用Autoware自带的传感器驱动 -->
      <arg name="launch_sensing_driver" value="false"/>
    </include>
</launch>

```
source install/setup.bash
ros2 launch sensor_driver senser_driver.launch.xml
```
# 重新编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select sample_sensor_kit_description sample_vehicle_description vehicle_info_util

## 通过下列命令查看根据上述参数计算的车子最小转弯半径：
ros2 run vehicle_info_util min_turning_radius_calculator.py


# 实车启动
```
source install/setup.bash
ros2 launch sensor_driver senser_driver.launch.xml
```
