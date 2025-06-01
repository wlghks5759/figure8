#include <rclcpp/rclcpp.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>

#include <iostream>
#include <chrono> //시간 duration : milliseconds, sec etc..
#include <thread> // 실행을 병렬로 수행할수 있게 해줌, sleep_for 도 사용가능
#include <cmath>
#include <future> // future는 비동기 작업의 결과를 나타내는 객체로, 비동기 작업이 완료되면 결과를 가져올 수 있음

class Figure8 : public rclcpp::Node {
    public:
        Figure8() : Node("figure8") {
            //parameter
            this->declare_parameter<std::string>("connection_url", "udp://:14540");
            this->declare_parameter<double>("x_amp", 10.0);
            this->declare_parameter<double>("y_amp", 5.0);
            this->declare_parameter<double>("omega", 0.5);
            this->declare_parameter<double>("duration", 0.0);
            this->declare_parameter<double>("height", 10.0);
            this->declare_parameter<double>("speed", 3.0);
            this->declare_parameter<double>("acceptance_radius", 1.0);

            double omega_val = this->get_parameter("omega").as_double();
            double calculated_duration = ((2.0 * M_PI) / omega_val) * 2;
            this->set_parameter(rclcpp::Parameter("duration", calculated_duration)); //x는 wt이니 y가 두바퀴돌아야 온전한 한사이클 완성
        }

}

int main() {
    return 0;
};
