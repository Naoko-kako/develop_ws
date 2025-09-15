#ifndef _CATCH_H_
#define _CATCH_H_

#include <iostream>
#include <vector>

// 自作クラス
#include "IP.hpp"
#include "UDP.hpp"

class Catch{
    public:
        static void set_Speed();
        static void Action(UDP &udp);
        static int getSpeedR();


    private:
        static int speed_r;
        static int speed_theata;
        static std::vector<int16_t> data; 

};


#endif //_CATCH_H_
