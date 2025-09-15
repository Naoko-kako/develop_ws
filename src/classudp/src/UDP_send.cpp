/*
　　UDPに値を送るクラス
*/

#include "include/UDP_send.hpp"


UDP_send::UDP_send(const std::string &ip, int port)
 : Node("udp_send"),udp_(ip, port),data(19,0)
{

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&UDP_send::udp_timer_callback, this)
    );
    RCLCPP_INFO(this->get_logger(),
                    "RITSMAYCAN initialized with IP: %s, Port: %d", ip.c_str(),
                    port);
}

void UDP_send::udp_timer_callback()
{
    
    for (size_t i = 0; i < data.size(); i++) {
        data[i] = 10; 
    }

    // UDPで送る
    udp_.send(data);
}

