#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

int main() 
{
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);  // 创建套接字
    if (sock < 0) {
        perror("Socket");
        return 1;
    }
    printf("Socket created successfully, sockfd = %d\n", sock);
    
    struct sockaddr_can addr;
    struct ifreq ifr;  //linux网路接口相关，在net/if.h中定义
    
    strcpy(ifr.ifr_name, "can1");
    ioctl(sock, SIOCGIFINDEX, &ifr);  // in out control; sock用于标识网络接口 请求码，SIOCGIFINDEX用于获取接口索引,接口的索引号填充到 ifr_ifindex 成员中,&ifr: 指向 ifreq 结构体的指针，用于传入接口名称并接收输出数据。

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;  

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {  // 绑定套接字
        perror("Bind");
        return 1;
    }
    struct can_frame frame;
    
    int nbytes = read(sock, &frame, sizeof(struct can_frame));  // 读取数据

    if (nbytes < 0) {
        perror("Read");
        return 1;
    }

    std::cout << "Received CAN frame" << std::endl;
    std::cout << "ID: " << std::hex << frame.can_id << " Data: ";
    
    for (int i = 0; i < frame.can_dlc; i++) {
        std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
    }
    
    std::cout << std::endl;

    close(sock);  // 关闭套接字
    return 0;
}

