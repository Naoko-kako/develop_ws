#include "catch.h"
#include "ps4.hpp"
#include <iostream>
#include <vector>


using namespace std;

//  初期化
int Catch::speed_r = 0;
int Catch::speed_theata = 0;

//  コンストラクタ
Catch::Catch()  {
    PS4_Listener::
}


void Catch::set_Speed() {
    cout << "R軸の速度を入力:";
    cin >> speed_r;
    cout << "θ軸の速度を入力:";
    cin >> speed_theata;
    cout << "R軸の速度" << speed_r << endl;
    cout << "θ軸の速度" << speed_theata << endl;
}

void Catch::PS4_main(UDP &udp) {
    if (PS.) {
            std::fill(data.begin(), data.end(), 0);          // 配列をゼロで埋める
            for (int attempt = 0; attempt < 10; attempt++) { // 10回試行
                udp_.send(data);                             // データ送信
                std::cout << "緊急停止！ 試行" << attempt + 1
                          << std::endl; // 試行回数を表示
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(100)); // 100msの遅延
            }
            rclcpp::shutdown();
        }

    if(CIRCLE){
        data[1] = speed_r;
    }
    if(TRIANGLE){
        data[2] = speed_theata;
    }
}


