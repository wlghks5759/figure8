#include <rclcpp/rclcpp.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>

#include <iostream>
#include <chrono> //시간 duration : milliseconds, sec etc..
#include <thread> // 실행을 병렬로 수행할수 있게 해줌, sleep_for 도 사용가능
#include <cmath>
#include <future> // future는 비동기 작업의 결과를 나타내는 객체로, 비동기 작업이 완료되면 결과를 가져올 수 있음
#include <sstream>

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

            connection_url = this->get_parameter("connection_url").as_string();
            x_amp = this->get_parameter("x_amp").as_double();
            y_amp = this->get_parameter("y_amp").as_double();
            height = this->get_parameter("height").as_double();
            speed = this->get_parameter("speed").as_double();
            acceptance_radius = this->get_parameter("acceptance_radius").as_double();
            duration = this->get_parameter("duration").as_double();

            RCLCPP_INFO(this->get_logger(), "URL: %s", connection_url.c_str());
            RCLCPP_INFO(this->get_logger(), "Settings: xamp: %f, yamp: %f, omega: %f, duration: %f, height: %f, speed: %f, acceptanceradius: %f"
                , x_amp, y_amp, omega_val, duration, height, speed, acceptance_radius);

            //별도 스레드에서 run mission 실행
            mission_thread_ = std::thread(&Figure8::run_mission, this);
        }
        ~Figure8() {
            if (mission_thread_.joinable()) {
                mission_thread_.join();
            }
            RCLCPP_INFO(this->get_logger(), "Figure8 shutdown.");
        }

    private:
        // 타입정의 부분
        std::thread mission_thread_;
        std::string connection_url;
        double x_amp;
        double y_amp;
        double omega;
        double duration;
        double height;
        double speed;
        double acceptance_radius;

        std::shared_ptr<mavsdk::Mavsdk> mavsdk_;
        std::shared_ptr<mavsdk::System> system_;
        std::shared_ptr<mavsdk::Action> action_;
        std::shared_ptr<mavsdk::Telemetry> telemetry_;
        std::shared_ptr<mavsdk::Offboard> offboard_;

        bool initial_health_ok_ = false;

        //Home
        bool home_position_set_ = false;
        double home_latitude_{0.0};
        double home_longitude_{0.0};
        float home_absolute_altitude_m_{0.0f};
        float home_relative_altitude_m_{0.0f};

        //Current position
        std::mutex position_mutex_;
        double current_latitude_{0.0};
        double current_longitude_{0.0};
        float current_relative_altitude_m_{0.0f};
        float current_absolute_altitude_m_{0.0f};

        std::atomic<bool> emergency_triggered_{false};

        void run_mission() {
            mavsdk_ = std::make_shared<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation});

            // MAVSDK 연결
            mavsdk::ConnectionResult connection_result = mavsdk_->add_any_connection(connection_url);

            if (connection_result != mavsdk::ConnectionResult::Success) {
                std::stringstream ss;
                ss << connection_result; // 스트림 연산자를 사용하여 ss에 문자열 형태로 기록
                RCLCPP_ERROR(this->get_logger(), "Connection failed: %s", ss.str().c_str()); // ss.str()은 std::string을 반환, .c_str()로 C-문자열 포인터 획득
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "MAVSDK connection established.");



            // 시스템 발견 대기
            RCLCPP_INFO(this->get_logger(), "Waiting for system to connect...");
            auto discovered_system_promise = std::promise<std::shared_ptr<mavsdk::System>>();
            auto discovered_system_future = discovered_system_promise.get_future();

            mavsdk_->subscribe_on_new_system([this, &discovered_system_promise]() {
                const auto systems = mavsdk_->systems();
                if (!systems.empty()) {
                    RCLCPP_INFO(this->get_logger(), "System discovered!");
                    discovered_system_promise.set_value(systems.front());
                    mavsdk_->subscribe_on_new_system(nullptr); // Unsubscribe
                }
            });

            if (discovered_system_future.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
                RCLCPP_ERROR(this->get_logger(), "No system discovered in 10 seconds.");
                rclcpp::shutdown();
                return;
            }
            system_ = discovered_system_future.get();

            action_ = std::make_shared<mavsdk::Action>(system_);
            telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
            offboard_ = std::make_shared<mavsdk::Offboard>(system_);

            // 텔레메트리 구독 (홈 위치, 현재 위치 등)
            telemetry_->subscribe_health_all_ok([this](bool all_ok){
                if(!initial_health_ok_ && all_ok) {
                    RCLCPP_INFO(this->get_logger(), "System health is OK.");
                    initial_health_ok_ = true;
                } else if (initial_health_ok_ && !all_ok) {
                    RCLCPP_WARN(this->get_logger(), "System health problem detected!");
                }
            });

            telemetry_->subscribe_home([this](mavsdk::Telemetry::Position home_pos){
                if(!home_position_set_){
                    home_latitude_ = home_pos.latitude_deg;
                    home_longitude_ = home_pos.longitude_deg;
                    home_absolute_altitude_m_ = home_pos.absolute_altitude_m;
                    home_relative_altitude_m_ = home_pos.relative_altitude_m; // Should be 0 at home
                    home_position_set_ = true;
                    RCLCPP_INFO(this->get_logger(), "Home position set: Lat=%.7f, Lon=%.7f, AbsAlt=%.2fm",
                                home_latitude_, home_longitude_, home_absolute_altitude_m_);
                    }
            });

            // current GPS
            telemetry_->subscribe_position([this](mavsdk::Telemetry::Position position) {
                std::lock_guard<std::mutex> lock(position_mutex_);
                current_latitude_ = position.latitude_deg;
                current_longitude_ = position.longitude_deg;
                current_relative_altitude_m_ = position.relative_altitude_m;
                current_absolute_altitude_m_ = position.absolute_altitude_m;
            });

            // 1. Pre-flight checks
            RCLCPP_INFO(this->get_logger(), "Waiting for vehicle to have a GPS fix and home position...");
            while (!telemetry_->health_all_ok() || !home_position_set_) {
                if (emergency_triggered_) { RCLCPP_WARN(this->get_logger(), "Emergency triggered during pre-flight checks. Aborting."); return; }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for GPS fix and home position...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            RCLCPP_INFO(this->get_logger(), "GPS fix and home position acquired.");


            // 2. Arm
            RCLCPP_INFO(this->get_logger(), "Arming...");
            const mavsdk::Action::Result arm_result = action_->arm();
            if (arm_result != mavsdk::Action::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Arming failed");
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Armed.");

            
            // 3. Takeoff
            RCLCPP_INFO(this->get_logger(), "Setting takeoff altitude to %.2f m", height);
            action_->set_takeoff_altitude(height);
            RCLCPP_INFO(this->get_logger(), "Taking off...");
            const mavsdk::Action::Result takeoff_result = action_->takeoff();
            if (takeoff_result != mavsdk::Action::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Takeoff failed");
                rclcpp::shutdown();
                return;
            }
        };

};

int main() {
    return 0;
};
