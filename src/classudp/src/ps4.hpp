#ifndef _PS4_H_
#define _PS4_H_

using namespace std;

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class PS4_Listener : public rclcpp::Node{
    public: 
        PS4_Listener();
    private:
        void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

#endif /* _PS4_H_* /

