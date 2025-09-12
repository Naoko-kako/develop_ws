#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

#include "ps4.hpp"

using namespace std;

PS4_Listener::PS4_Listener()
        PS4_Listener() : Node("catch2025_rmc")
        {
            subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10,
                bind(&PS4_Listener::ps4_listener_callback, this,
                      placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "joy獲得");
        }


void PS4_Listener::ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{   
     // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        float LS_X = -1 * msg->axes[0];
        float LS_Y = msg->axes[1];
        float RS_X = -1 * msg->axes[3];
        float RS_Y = msg->axes[4];

        bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        bool R1 = msg->buttons[5];

        float L2 = (-1 * msg->axes[2] + 1) / 2;
        float R2 = (-1 * msg->axes[5] + 1) / 2;

        bool SHARE = msg->buttons[8];
        bool OPTION = msg->buttons[9];
        bool PS = msg->buttons[10];

        bool L3 = msg->buttons[11];
        bool R3 = msg->buttons[12];

        // 

}
        

        