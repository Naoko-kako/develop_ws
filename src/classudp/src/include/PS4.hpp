/*
　　コントローラーの値を取るクラス
*/


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// 自作クラス
#include "IP.hpp"
#include "UDP.hpp"
#include "catch.h"

class PS4_Listener : public rclcpp::Node 
{
    public:
        PS4_Listener(const std::string &ip, int port);

    private:
        void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
        UDP udp_;
        std::vector<int16_t> data;
};