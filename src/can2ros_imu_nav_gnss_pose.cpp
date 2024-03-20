#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cmath>

struct vector3d {double x, y, z;
};

vector3d LatLongAltToEcef(const double latitude, const double longitude, const double altitude) {
// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates

constexpr double a = 6378137.0; // semi-major axis, equator to center.
constexpr double f = 1. / 298.257223563;
constexpr double b = a * (1. - f); // semi-minor axis, pole to center.
constexpr double a_squared = a * a;
constexpr double b_squared = b * b;
constexpr double e_squared = (a_squared - b_squared) / a_squared;

const double sin_phi = std::sin(latitude * M_PI / 180.0);
const double cos_phi = std::cos(latitude * M_PI / 180.0);
const double sin_lambda = std::sin(longitude * M_PI / 180.0);
const double cos_lambda = std::cos(longitude * M_PI / 180.0);

const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
const double x = (N + altitude) * cos_phi * cos_lambda;
const double y = (N + altitude) * cos_phi * sin_lambda;
const double z = (b_squared / a_squared * N + altitude) * sin_phi;

return {x, y, z};
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("can_data_processor");
    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    auto navsat_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("navsat_data", 10);
    auto pose_from_gnss_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("pose_from_gnss", 10);
    
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
    geometry_msgs::msg::PoseStamped pose_from_gnss_msg;
    
    
    

    
    
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
                //navsat_msg.header.frame_id = "?";
                navsat_msg.header.stamp = node->get_clock()->now();
                navsat_pub->publish(navsat_msg); 
                
                auto ecef = LatLongAltToEcef(navsat_msg.latitude, navsat_msg.longitude, navsat_msg.altitude);
                pose_from_gnss_msg.pose.position.x = ecef.x;
                pose_from_gnss_msg.pose.position.y = ecef.y;
                pose_from_gnss_msg.pose.position.z = ecef.z;
                pose_from_gnss_msg.pose.orientation.x = 0.0;
                pose_from_gnss_msg.pose.orientation.y = 0.0;
                pose_from_gnss_msg.pose.orientation.z = 0.0;
                pose_from_gnss_msg.pose.orientation.w = 1.0;
                pose_from_gnss_msg.header.stamp = node->get_clock()->now();
                //pose_from_gnss_msg.header.frame_id = "some_frame";
                pose_from_gnss_pub->publish(pose_from_gnss_msg);
                
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
                imu_msg.header.stamp = node->get_clock()->now();
                //imu_msg.header.frame_id = "?";
                imu_pub->publish(imu_msg);
                break;
            }
        }
    }

    close(sock);
    rclcpp::shutdown();
    return 0;
}

