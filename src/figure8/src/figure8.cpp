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

// 지구 반경 (미터)
const double EARTH_RADIUS_M = 6371000.0;

// 도 단위를 라디안으로 변환
double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

// 두 GPS 좌표 간의 거리 계산 (Haversine formula)
double calculate_distance_haversine(double lat1, double lon1, double lat2, double lon2) {
    double dLat = to_radians(lat2 - lat1);
    double dLon = to_radians(lon2 - lon1);

    lat1 = to_radians(lat1);
    lat2 = to_radians(lat2);

    double a = sin(dLat / 2) * sin(dLat / 2) +
               sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS_M * c;
}

// 두 GPS 좌표 간의 방위각 계산 (초기 방위각)
// 두 GPS 좌표 간의 방위각 계산 (초기 방위각)
double calculate_bearing(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {


    double dLon_deg = lon2_deg - lon1_deg;
    double dLon_rad = to_radians(dLon_deg);

    double lat1_rad = to_radians(lat1_deg);
    double lat2_rad = to_radians(lat2_deg); // 목표 위도를 라디안으로 변환


    // 수정된 계산 (일반적인 방위각 공식)
    // https://www.movable-type.co.uk/scripts/latlong.html
    double y_calc = sin(dLon_rad) * cos(lat2_rad);
    double x_calc = cos(lat1_rad) * sin(lat2_rad) -
                    sin(lat1_rad) * cos(lat2_rad) * cos(dLon_rad);


    double bearing_rad_calc = atan2(y_calc, x_calc);
    double bearing_deg_calc = fmod((bearing_rad_calc * 180.0 / M_PI + 360.0), 360.0); // 0-360도 범위로 정규화


    return bearing_deg_calc;
}


void convert_global_to_ned(double home_lat_deg, double home_lon_deg,
                           double current_lat_deg, double current_lon_deg,
                           double& north_m, double& east_m) {
    // 1. Home과 현재 위치 간의 직선 거리 계산 (Haversine 공식 사용)
    double distance = calculate_distance_haversine(home_lat_deg, home_lon_deg, current_lat_deg, current_lon_deg);

    // 2. Home에서 현재 위치로의 초기 방위각 계산 (calculate_bearing 함수 사용)
    // calculate_bearing 함수는 이미 0-360도 범위의 각도를 반환합니다.
    double bearing_deg = calculate_bearing(home_lat_deg, home_lon_deg, current_lat_deg, current_lon_deg);
    double bearing_rad = to_radians(bearing_deg); // 라디안으로 변환

    // 3. 거리와 방위각을 이용하여 North(X)와 East(Y) 컴포넌트 계산
    // North = 거리 * cos(방위각)
    // East = 거리 * sin(방위각)
    // 수학/물리에서 0도(동쪽)부터 시작하여 반시계 방향으로 각도가 증가하는 경우가 많지만,
    // 항법에서는 0도(북쪽)부터 시작하여 시계 방향으로 각도가 증가하는 경우가 흔합니다.
    // Movable-type의 bearing은 0=North, positive=clockwise입니다.
    // 따라서 North는 cos, East는 sin을 그대로 사용합니다.
    north_m = distance * std::cos(bearing_rad);
    east_m = distance * std::sin(bearing_rad);
}

class Figure8 : public rclcpp::Node {
    public:
        Figure8() : Node("figure8") {
            //parameter
            this->declare_parameter<std::string>("connection_url", "udp://:14540");
            this->declare_parameter<double>("x_amp", 100.0);
            this->declare_parameter<double>("y_amp", 50.0);
            this->declare_parameter<double>("omega", 0.5);
            this->declare_parameter<double>("duration", 0.0);
            this->declare_parameter<double>("height", 10.0);
            this->declare_parameter<double>("speed", 1.0);
            this->declare_parameter<double>("acceptance_radius", 2.0);

            omega_val = this->get_parameter("omega").as_double();
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

        enum class MissionState {
            INITIAL,
            PRE_FLIGHT_CHECKS,
            ARMING,
            TAKING_OFF,
            STARTING_OFFBOARD,
            FLYING_FIGURE8,
            LANDING,
            FINISHED
        };
        
        MissionState current_state_ = MissionState::INITIAL;
        
        // 타입정의 부분
        std::thread mission_thread_;
        std::string connection_url;
        double x_amp;
        double y_amp;
        double omega_val;
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

        //for NED position
        float current_north_m_{0.0f};
        float current_east_m_{0.0f};
        float current_down_m_{0.0f}; 

        std::atomic<bool> emergency_triggered_{false};

        //orbit_vector
        std::vector<mavsdk::Offboard::PositionNedYaw> figure8_path_;

        void figure8_path_generator() {
            figure8_path_.clear();
            int num_point = 100;
            double num_points_double = static_cast<double>(num_point);

            for (int i = 0; i < num_point; i++) {
                double t = (static_cast<double>(i) / num_points_double) * duration; // 0 to duration
                float x = x_amp * std::sin(omega_val * t);
                float y = y_amp * std::sin(2.0 * omega_val * t);
                float z_ned = -static_cast<float>(height);

                // 속도 벡터 계산 (곡선의 미분)
                float dx_dt = x_amp * omega_val * std::cos(omega_val * t);
                float dy_dt = y_amp * 2.0 * omega_val * std::cos(2.0 * omega_val * t);

                // Yaw 각도 계산 (진행 방향) arctan()
                float yaw_rad = std::atan2(dy_dt, dx_dt);
                float yaw_deg = yaw_rad * 180.0f / M_PI;

                mavsdk::Offboard::PositionNedYaw point;
                point.north_m = x;
                point.east_m = y;
                point.down_m = z_ned;
                point.yaw_deg = yaw_deg;

                figure8_path_.push_back(point);
            }
        }


        // 작동과정
        void run_mission() {
            // --- 1. MAVSDK 초기화 및 연결 ---
            mavsdk_ = std::make_shared<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation});
            mavsdk::ConnectionResult connection_result = mavsdk_->add_any_connection(connection_url);
            if (connection_result != mavsdk::ConnectionResult::Success) {
                RCLCPP_ERROR(this->get_logger(), "Connection failed");
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "MAVSDK connection established.");

            // --- 2. 시스템 발견 및 플러그인 초기화 ---
            auto discovered_system_promise = std::promise<std::shared_ptr<mavsdk::System>>();
            auto discovered_system_future = discovered_system_promise.get_future();
            mavsdk_->subscribe_on_new_system([this, &discovered_system_promise]() {
                if (!mavsdk_->systems().empty()) {
                    RCLCPP_INFO(this->get_logger(), "System discovered!");
                    discovered_system_promise.set_value(mavsdk_->systems().front());
                    mavsdk_->subscribe_on_new_system(nullptr);
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

            // --- 3. 텔레메트리 구독 ---
            // 이 부분은 기존 코드와 동일해야 합니다. 이미 작성되어 있다면 수정할 필요 없습니다.
            telemetry_->subscribe_health_all_ok([this](bool all_ok){
                if(!initial_health_ok_ && all_ok) {
                    RCLCPP_INFO(this->get_logger(), "System health is OK.");
                    initial_health_ok_ = true;
                } else if (initial_health_ok_ && !all_ok) {
                    RCLCPP_WARN(this->get_logger(), "System health problem detected!");
                    // 필요하다면 여기서 emergency_triggered_ 플래그를 설정할 수 있습니다.
                }
            });

            telemetry_->subscribe_home([this](mavsdk::Telemetry::Position home_pos){
                if(!home_position_set_){
                    home_latitude_ = home_pos.latitude_deg;
                    home_longitude_ = home_pos.longitude_deg;
                    home_absolute_altitude_m_ = home_pos.absolute_altitude_m;
                    home_relative_altitude_m_ = home_pos.relative_altitude_m;
                    home_position_set_ = true;
                    RCLCPP_INFO(this->get_logger(), "Home position set: Lat=%.7f, Lon=%.7f, AbsAlt=%.2fm",
                                home_latitude_, home_longitude_, home_absolute_altitude_m_);
                    }
            });

            telemetry_->subscribe_position([this](mavsdk::Telemetry::Position position) {
                std::lock_guard<std::mutex> lock(position_mutex_);
                current_latitude_ = position.latitude_deg;
                current_longitude_ = position.longitude_deg;
                current_relative_altitude_m_ = position.relative_altitude_m;
                current_absolute_altitude_m_ = position.absolute_altitude_m;

                if (home_position_set_) { 
                    double north_tmp, east_tmp;
                    convert_global_to_ned(home_latitude_, home_longitude_,
                                        current_latitude_, current_longitude_,
                                        north_tmp, east_tmp);
                    current_north_m_ = static_cast<float>(north_tmp);
                    current_east_m_ = static_cast<float>(east_tmp);
                    current_down_m_ = -current_relative_altitude_m_;
                }
            });


            // --- 4. 메인 제어 루프 (상태 머신) ---
            current_state_ = MissionState::PRE_FLIGHT_CHECKS;
            size_t figure8_idx = 0;

            while (current_state_ != MissionState::FINISHED && rclcpp::ok() && !emergency_triggered_) {
                
                switch (current_state_) {
                    case MissionState::PRE_FLIGHT_CHECKS:
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: PRE_FLIGHT_CHECKS");
                        if (telemetry_->health_all_ok() && home_position_set_) {
                            RCLCPP_INFO(this->get_logger(), "GPS fix and home position acquired.");
                            current_state_ = MissionState::ARMING;
                        }
                        break;

                    case MissionState::ARMING:
                        {
                            // Arming을 위한 변수들
                            static int arming_attempts = 0;
                            const int max_arming_attempts = 10; // 최대 10번 (5초) 시도
                            
                            RCLCPP_INFO_THROTTLE(
                                this->get_logger(), 
                                *this->get_clock(), 
                                1000, 
                                "State: ARMING (Attempt %d/%d)", 
                                arming_attempts + 1, 
                                max_arming_attempts
                            );

                            const auto arm_result = action_->arm();
                            if (arm_result == mavsdk::Action::Result::Success) {
                                RCLCPP_INFO(this->get_logger(), "Armed successfully.");
                                current_state_ = MissionState::TAKING_OFF;
                            } else {
                                arming_attempts++;
                                if (arming_attempts >= max_arming_attempts) {
                                    current_state_ = MissionState::LANDING; 
                                }
                                // 실패 시 바로 다음 루프에서 재시도 (50ms 간격)
                            }
                        }
                        break;

                    case MissionState::TAKING_OFF:
                        {
                            static bool takeoff_triggered = false;
                            // 타임아웃을 위한 시간 기록
                            static std::chrono::steady_clock::time_point takeoff_start_time;

                            if (!takeoff_triggered) {
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: TAKING_OFF - Sending command");
                                action_->set_takeoff_altitude(height);
                                const auto takeoff_result = action_->takeoff();
                                if (takeoff_result == mavsdk::Action::Result::Success) {
                                    RCLCPP_INFO(this->get_logger(), "Takeoff command sent.");
                                    takeoff_triggered = true;
                                    takeoff_start_time = std::chrono::steady_clock::now(); // 이륙 시작 시간 기록
                                } else {
                                    RCLCPP_ERROR(this->get_logger(), "Takeoff failed");
                                    current_state_ = MissionState::LANDING;
                                }
                            } else {
                                // 이륙 명령을 보낸 후
                                float current_alt;
                                bool in_air = telemetry_->in_air();
                                {
                                    std::lock_guard<std::mutex> lock(position_mutex_);
                                    current_alt = current_relative_altitude_m_;
                                }
                                
                                RCLCPP_INFO_THROTTLE(
                                    this->get_logger(), *this->get_clock(), 1000, 
                                    "State: TAKING_OFF - In air: %s, Altitude: %.2f / %.2f m", 
                                    in_air ? "true" : "false", current_alt, height
                                );
                                
                                // 조건 1: 목표 고도의 85% 이상에 도달했는가? (조건 완화)
                                if (current_alt >= height * 0.85f && in_air) {
                                    RCLCPP_INFO(this->get_logger(), "Takeoff complete. Reached altitude: %.2f m", current_alt);
                                    current_state_ = MissionState::STARTING_OFFBOARD;
                                }

                                // 조건 2: 이륙을 시작한 지 너무 오래되지는 않았는가? (타임아웃)
                                auto elapsed = std::chrono::steady_clock::now() - takeoff_start_time;
                                if (elapsed > std::chrono::seconds(30)) { // 30초 타임아웃
                                    RCLCPP_ERROR(this->get_logger(), "Takeoff timeout! Failed to reach altitude.");
                                    current_state_ = MissionState::LANDING;
                                }
                            }
                        }
                        break;

                    case MissionState::STARTING_OFFBOARD:
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: STARTING_OFFBOARD");
                        
                        // Offboard 모드 시작 전, 현재 위치를 유지하도록 초기 Setpoint를 몇 번 보냅니다.
                        // 이는 PX4가 Offboard 모드로 부드럽게 전환할 준비를 할 시간을 줍니다.
                        {
                            mavsdk::Offboard::PositionNedYaw initial_setpoint{};
                            initial_setpoint.north_m = current_north_m_; // 현재 위치를 목표로 설정
                            initial_setpoint.east_m = current_east_m_;
                            initial_setpoint.down_m = -height;
                            initial_setpoint.yaw_deg = telemetry_->attitude_euler().yaw_deg;
                            offboard_->set_position_ned(initial_setpoint);
                        }

                        // EKF가 위치를 확실히 인지할 시간을 주기 위해 짧은 대기
                        std::this_thread::sleep_for(std::chrono::milliseconds(100));

                        {
                            auto offboard_start_result = offboard_->start();
                            if (offboard_start_result == mavsdk::Offboard::Result::Success) {
                                RCLCPP_INFO(this->get_logger(), "Offboard mode started successfully.");
                                
                                // Offboard 시작 후, 실제로 비행 작업을 시작하기 전에
                                // 현재 위치를 유지하는 명령을 보내며 잠시 대기합니다.
                                // 이 과정이 PX4가 FlightTaskOffboard를 활성화하는 데 매우 중요합니다.
                                static bool stabilized = false;
                                static std::chrono::steady_clock::time_point start_time;
                                if (!stabilized) {
                                    start_time = std::chrono::steady_clock::now();
                                    stabilized = true;
                                }

                                if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(2)) {
                                    RCLCPP_INFO(this->get_logger(), "Offboard stabilized. Starting Figure 8 path.");
                                    figure8_path_generator(); // 경로 생성은 여기서 한 번만
                                    current_state_ = MissionState::FLYING_FIGURE8;
                                }

                            } else {
                                
                                current_state_ = MissionState::LANDING;
                            }
                        }
                        break;

                    case MissionState::FLYING_FIGURE8:
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: FLYING_FIGURE8 - To Waypoint %zu", figure8_idx);
                        {
                            const auto& target_point = figure8_path_[figure8_idx];
                            offboard_->set_position_ned(target_point);

                            float current_n, current_e;
                            {
                                std::lock_guard<std::mutex> lock(position_mutex_);
                                current_n = current_north_m_;
                                current_e = current_east_m_;
                            }

                            // 수평 거리만 계산하여 웨이포인트 도달 여부 판단
                            float distance_to_target = std::sqrt(
                                std::pow(current_n - target_point.north_m, 2) +
                                std::pow(current_e - target_point.east_m, 2)
                            );

                            if (distance_to_target < acceptance_radius) {
                                RCLCPP_INFO(this->get_logger(), "Reached setpoint %zu", figure8_idx);
                                figure8_idx++;
                                if (figure8_idx >= figure8_path_.size()) {
                                    RCLCPP_INFO(this->get_logger(), "Figure 8 cycle complete. Landing.");
                                    current_state_ = MissionState::LANDING;
                                }
                            }
                        }
                        break;

                    case MissionState::LANDING:
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: LANDING");
                        {
                            static bool land_triggered = false;
                            if (!land_triggered) {
                                if (offboard_->is_active()) {
                                    offboard_->stop();
                                    RCLCPP_INFO(this->get_logger(), "Offboard mode stopped for landing.");
                                }
                                const auto land_result = action_->land();
                                if (land_result == mavsdk::Action::Result::Success) {
                                    RCLCPP_INFO(this->get_logger(), "Landing command sent.");
                                    land_triggered = true;
                                } else {
                                    RCLCPP_ERROR(this->get_logger(), "Landing failed");
                                }
                            }
                            
                            if (!telemetry_->in_air()) {
                                RCLCPP_INFO(this->get_logger(), "Landed.");
                                const auto disarm_result = action_->disarm();
                                if (disarm_result == mavsdk::Action::Result::Success) {
                                    RCLCPP_INFO(this->get_logger(), "Disarmed.");
                                }
                                current_state_ = MissionState::FINISHED;
                            }
                        }
                        break;
                    
                    default:
                        // 혹시 모를 예외 상황
                        break;
                }

                // 루프는 항상 일정한 주기로 돌아야 합니다. Offboard 타임아웃 방지.
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20Hz
            }

            // --- 5. 루프 종료 후 정리 ---
            RCLCPP_INFO(this->get_logger(), "Mission thread finished.");
            rclcpp::shutdown();
        }


};



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // ROS 2 초기화
    auto node = std::make_shared<Figure8>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
