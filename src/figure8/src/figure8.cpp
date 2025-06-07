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
            this->declare_parameter<double>("acceptance_radius", 0.5);

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


            mavsdk::Action::Result arm_result;
            mavsdk::Action::Result takeoff_result;
            mavsdk::Offboard::Result offboard_start_result;
            mavsdk::Offboard::VelocityNedYaw initial_setpoint{};
            
            size_t curr_idx = 0;




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

                //lat, lon to NED
                if (home_position_set_) { 
                    double north_tmp, east_tmp;
                    convert_global_to_ned(home_latitude_, home_longitude_,
                                          current_latitude_, current_longitude_,
                                          north_tmp, east_tmp);
                    current_north_m_ = static_cast<float>(north_tmp);
                    current_east_m_ = static_cast<float>(east_tmp);
                    // Down은 상대 고도의 음수값 (NED 좌표계의 Down은 아래가 양수)
                    current_down_m_ = -current_relative_altitude_m_;
                }
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
            arm_result = action_->arm();
            if (arm_result != mavsdk::Action::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Arming failed");
                goto land_sequence; // 실패 시 착륙 시도
            }
            RCLCPP_INFO(this->get_logger(), "Armed.");

            
            // 3. Takeoff
            RCLCPP_INFO(this->get_logger(), "Setting takeoff altitude to %.2f m", height);
            action_->set_takeoff_altitude(height);
            RCLCPP_INFO(this->get_logger(), "Taking off...");
            takeoff_result = action_->takeoff();
            if (takeoff_result != mavsdk::Action::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Takeoff failed");
                goto land_sequence; // 실패 시 착륙 시도
            }


            while (true) {
                if (emergency_triggered_) {RCLCPP_WARN(this->get_logger(), "Emergency triggered during takeoff. Attempting to land."); 
                    action_->land();
                    rclcpp::shutdown();
                    return;
                }
                float curr;
                {
                    std::lock_guard<std::mutex> lock(position_mutex_);
                    curr = current_relative_altitude_m_;
                }
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Current altitude: %.2f m", curr);
                if (curr >= height * 0.98f) { // 95% 도달 시
                    RCLCPP_INFO(this->get_logger(), "Takeoff complete. Reached altitude: %.2f m", curr);
                    break;
                    }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
            if (emergency_triggered_) {
            // 비상 착륙 코드
            RCLCPP_INFO(this->get_logger(), "Emergency triggered, initiating landing sequence");
            }

            // 4. Offboard mode
            RCLCPP_INFO(this->get_logger(), "Preparing for Offboard mode to target...");
            figure8_path_generator();

            
            initial_setpoint.north_m_s = 0.0f;
            initial_setpoint.east_m_s = 0.0f;
            initial_setpoint.down_m_s = 0.0f;
            initial_setpoint.yaw_deg = telemetry_->attitude_euler().yaw_deg;
            offboard_->set_velocity_ned(initial_setpoint);
            RCLCPP_INFO(this->get_logger(), "Initial setpoint sent (hover).");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            offboard_start_result = offboard_->start();
            if (offboard_start_result != mavsdk::Offboard::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Offboard start failed");
                goto land_sequence; // 실패 시 착륙 시도
                }

            RCLCPP_INFO(this->get_logger(), "Offboard mode started.");


            // Offboard 모드로 이동
            while (rclcpp::ok() && !emergency_triggered_) {

                // 목표 위치 설정
                const auto& target_point = figure8_path_[curr_idx];

                float current_n, current_e, current_d;
                {
                    std::lock_guard<std::mutex> lock(position_mutex_);
                    current_n = current_north_m_;
                    current_e = current_east_m_;
                    current_d = current_down_m_;
                }

                offboard_->set_position_ned(target_point);
                
                RCLCPP_INFO(this->get_logger(), "Setpoint %zu: North=%.2f, East=%.2f, Down=%.2f, Yaw=%.2f",
                            curr_idx, target_point.north_m, target_point.east_m, target_point.down_m, target_point.yaw_deg);

                float distance_to_target = std::sqrt(
                    std::pow(current_n - target_point.north_m, 2) +
                    std::pow(current_e - target_point.east_m, 2) +
                    std::pow(current_d - target_point.down_m, 2) // 고도도 포함하여 3D 거리 계산
                );


                if (distance_to_target < acceptance_radius) {
                    RCLCPP_INFO(this->get_logger(), "Reached setpoint %zu", curr_idx);

                    std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
                    curr_idx++;

                    if (curr_idx >= figure8_path_.size()) {
                        RCLCPP_INFO(this->get_logger(), "Completed one Figure 8 cycle. Restarting from beginning.");
                        curr_idx = 0; 
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(50));

            }
            // 루프 종료 시 (rclcpp::ok()가 false이거나 emergency_triggered_가 true)
            RCLCPP_INFO(this->get_logger(), "Figure 8 mission loop ended. Proceeding to landing.");
            
            // Offboard 모드 중지 (명시적으로 중지하는 것이 좋음)
            if (offboard_->is_active()) {
                RCLCPP_INFO(this->get_logger(), "Stopping Offboard mode explicitly.");
                auto stop_result = offboard_->stop();
                if (stop_result != mavsdk::Offboard::Result::Success) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to stop offboard mode gracefully");
                }
            }

            goto land_sequence;


        land_sequence: // 착륙 시퀀스 시작점
            if (emergency_triggered_) {
                RCLCPP_WARN(this->get_logger(), "Emergency Land initiated!");
            }

            // 5. Landing
            RCLCPP_INFO(this->get_logger(), "Landing...");
            const mavsdk::Action::Result land_result = action_->land();
            if (land_result != mavsdk::Action::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Landing failed");
                // 실패해도 일단 계속 진행하여 종료 시도
            }

            // 착륙 완료 대기
            while (telemetry_->in_air()) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for landing to complete...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (emergency_triggered_ && !telemetry_->in_air()){ // 이미 착륙했는데 비상 플래그가 켜진 경우
                    break;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Landed.");

            // Disarm
            RCLCPP_INFO(this->get_logger(), "Disarming...");
            const mavsdk::Action::Result disarm_result = action_->disarm();
            if (disarm_result != mavsdk::Action::Result::Success) {
                RCLCPP_ERROR(this->get_logger(), "Disarming failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "Disarmed.");
            }


            RCLCPP_INFO(this->get_logger(), "Mission complete.");
            rclcpp::shutdown(); // ROS 노드 종료
        }
};



int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // ROS 2 초기화
    auto node = std::make_shared<Figure8>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
