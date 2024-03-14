#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("can_data_processor");
    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    auto navsat_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("navsat_data", 10);

    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("Socket");
        return 1;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can1");
    ioctl(sock, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return 1;
    }

    struct can_frame frame;
    sensor_msgs::msg::Imu imu_msg;
    sensor_msgs::msg::NavSatFix navsat_msg;

    while (rclcpp::ok()) {
        int nbytes = read(sock, &frame, sizeof(struct can_frame));
        
        if (nbytes < 0) {
            perror("Read");
            break;
        }

        switch (frame.can_id) {
            case 0x20B: {
                int32_t lat, lon;
                memcpy(&lat, frame.data, sizeof(int32_t));
                memcpy(&lon, frame.data + 4, sizeof(int32_t));
                navsat_msg.latitude = lat * 1e-7;
                navsat_msg.longitude = lon * 1e-7;
                break;
            }
            case 0x30B: {
                int32_t alt;
                memcpy(&alt, frame.data, sizeof(int32_t));
                navsat_msg.altitude = alt * 1e-3;
                navsat_pub->publish(navsat_msg);
                break;
            }
            case 0x50B: {
                int32_t ax, ay;
                memcpy(&ax, frame.data, sizeof(int32_t));
                memcpy(&ay, frame.data + 4, sizeof(int32_t));
                imu_msg.angular_velocity.x = ax * 1e-5;
                imu_msg.angular_velocity.y = ay * 1e-5;
                break;
            }
            case 0x60B: {
                int32_t vx, vy;
                memcpy(&vx, frame.data, sizeof(int32_t));
                memcpy(&vy, frame.data + 4, sizeof(int32_t));
                imu_msg.linear_acceleration.x = vx * 1e-5;
                imu_msg.linear_acceleration.y = vy * 1e-5;
                break;
            }
            case 0x70B: {
                int32_t vz, az;
                memcpy(&vz, frame.data, sizeof(int32_t));
                memcpy(&az, frame.data + 4, sizeof(int32_t));
                imu_msg.linear_acceleration.z = vz * 1e-5;
                imu_msg.angular_velocity.z = az * 1e-5;
                imu_pub->publish(imu_msg);
                break;
            }
        }
    }

    close(sock);
    rclcpp::shutdown();
    return 0;
}

