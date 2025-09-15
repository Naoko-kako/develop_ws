#include "include/catch.h"

//  初期化
int Catch::speed_r = 0;
int Catch::speed_theata = 0;

std::vector<int16_t> Catch::data(19,0); 


void Catch::set_Speed() 
{
    std::cout << "R軸の速度を入力:";
    std::cin >> speed_r;
    std::cout << "θ軸の速度を入力:";
    std::cin >> speed_theata;
    std::cout << "R軸の速度" << speed_r << std::endl;
    std::cout << "θ軸の速度" << speed_theata << std::endl;
}



/*
マイコンに送信される配列"data"
debug: マイコンのprintfを有効化, MD: モータードライバー, TR: トランジスタ
| data[n] | 詳細 | 範囲 |
| ---- | ---- | ---- |
| data[0] | debug | 0 or 1 |
| data[1] | MD1 | -100 ~ 100 |
| data[2] | MD2 | -100 ~ 100 |
| data[3] | MD3 | -100 ~ 100 |
| data[4] | MD4 | -100 ~ 100 |
| data[5] | MD5 | -100 ~ 100 |
| data[6] | MD6 | -100 ~ 100 |
| data[7] | Servo1 | 0 ~ 180 |
| data[8] | Servo2 | 0 ~ 180 |
| data[9] | Servo3 | 0 ~ 180 |
| data[10] | Servo4 | 0 ~ 180 |
| data[11] | TR1 | 0 or 1|  //VGOAL
| data[12] | TR2 | 0 or 1|
| data[13] | TR3 | 0 or 1|  //ポンプ１
| data[14] | TR4 | 0 or 1|   //ポンプ２
| data[15] | TR5 | 0 or 1|  //シリンダ
| data[16] | TR6 | 0 or 1|
| data[17] | TR7 | 0 or 1|
| data[18] | TR8 | 0 or 1|

*/

void Catch::Action(UDP &udp) 
{
    std::cout<< "Hello," << std::endl;
    udp.send(data);

}


int Catch::getSpeedR() {
    return speed_r;
}

