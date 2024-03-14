#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

int main() {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("Socket");
        return 1;
    }
    
    std::cout << "socket is" << sock << std::endl;
    
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
    while (true) {  // 循环读取
        int nbytes = read(sock, &frame, sizeof(struct can_frame));
        
        
        if (nbytes < 0) {
            perror("Read");
            break;  // 如果读取出错，跳出循环
        }
        
        std::cout << "Received CAN frame from interface can1" << std::endl;
        std::cout << "ID: " << std::hex << frame.can_id << " Data: ";
        for (int i = 0; i < frame.can_dlc; i++) {
            std::cout << std::hex << +frame.data[i] << " ";
        }
        std::cout << std::endl;
    }

    close(sock);
    return 0;
}


