#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("can2rostopic");
    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);

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

    while (rclcpp::ok()) {
        int nbytes = read(sock, &frame, sizeof(struct can_frame));
        
        if (nbytes < 0) {
            perror("Read");
            break;  
        }

       
        if (frame.can_id == 0x50B) {
            // Assuming the data is in float format
            memcpy(&imu_msg.angular_velocity.x, frame.data, sizeof(float));
            memcpy(&imu_msg.angular_velocity.y, frame.data + 4, sizeof(float));
        } else if (frame.can_id == 0x60B) {
            memcpy(&imu_msg.linear_acceleration.x, frame.data, sizeof(float));
            memcpy(&imu_msg.linear_acceleration.y, frame.data + 4, sizeof(float));
        } else if (frame.can_id == 0x70B) {
            memcpy(&imu_msg.linear_acceleration.z, frame.data, sizeof(float));
            memcpy(&imu_msg.angular_velocity.z, frame.data + 4, sizeof(float));

           
            imu_pub->publish(imu_msg);
        }
    }

    close(sock);
    rclcpp::shutdown();
    return 0;
}

