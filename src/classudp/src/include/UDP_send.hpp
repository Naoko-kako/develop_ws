/*
　　UDPに値を送るクラス
*/

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

// 自作クラス
#include "IP.hpp"
#include "UDP.hpp"


class UDP_send :public rclcpp::Node
{
    public:
        UDP_send(const std::string &ip, int port);

    private:
        void udp_timer_callback();
        UDP udp_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<int16_t> data;
};
