/*[catchrobocontest-2025]
    [rits may can]
        [test]  */

#include <iostream>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>

// UDP通信用クラス
#include "include/IP.hpp"
#include "include/UDP.hpp"

// 自作クラス
#include "catch.h"
#include "ps4.hpp" 

// マイコン側のデバッグ
#define MC_PRINTF 0 

// マイコンに送信される配列
//std::vector<int16_t> data(19, 0);



int main (int argc, char *argv[]){
    rclcpp::init(argc, argv);
    //仮。後でフィグレットのクラスを作る
    cout << "mainloop" << endl;
    rclcpp::executors::SingleThreadedExecutor exec;
    auto ps4_listener = std::make_shared<PS4_Listener>();
    auto catchtest = std::make_shared<Catch>(IP_TEST, PORT_TEST);
    exec.add_node(ps4_listener);
    exec.add_node(catchtest);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
