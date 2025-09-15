/*
RRST-NHK-Project 2025 キャチロボ２回生チーム
RITS MAY CAN

--機構の制御
*/


//ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

//自作クラス
#include "include/IP.hpp"
#include "include/UDP.hpp"

#include "include/PS4.hpp" //ps4の値を受け取る+udp通信
#include "include/catch.h" //機構の制御

#define MC_PRINTF 0 // マイコン側のprintfを無効化・有効化(0 or 1

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);

    // figletでノード名を表示
    std::string figletout = "figlet RRST CATCH2025 RITSMAYCAN";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }
    

    rclcpp::executors::SingleThreadedExecutor exec;
    
    auto ps4_listener = std::make_shared<PS4_Listener>(IP_RITSMAYCAN, PORT_RITSMAYCAN);
    exec.add_node(ps4_listener);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}