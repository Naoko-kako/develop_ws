#ifndef _CATCH_H_
#define _CATCH_H_

using namespace std;


class Catch{
    public:
        Catch();    //コンストラクタ
        ~Catch();   //デストラクタ
        static void set_Speed();

    private:
        static int r;
        static int theata;
        vector<int16_t> data(19, 0); 

};

//CATCH --ROS2--> main

class Catch_Publisher : public rclcpp::Node{
    public:
        Catch_Publisher();
    private:
        void timer_callback();
        rclcpp::Publisher<>::SharedPtr publisher_;
        rclcpp::Timerbase::SharedPtr timer_;
};

class Catch_Listener : public rclcpp::Node{
    public: 
        Catch_Listener(const std::string &ip, int port);
    private:
        void Catch_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};



#endif /* _CATCH_H_* /
