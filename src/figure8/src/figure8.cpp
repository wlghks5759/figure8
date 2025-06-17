#include <rclcpp/rclcpp.hpp>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <future>
#include <vector>
#include <sstream>
#include <iomanip>

// --- 좌표 변환 함수 (변경 없음) ---
const double EARTH_RADIUS_M = 6371000.0;
double to_radians(double degrees) { return degrees * M_PI / 180.0; }
double to_degrees(double radians) { return radians * 180.0 / M_PI; }
double calculate_distance_haversine(double lat1, double lon1, double lat2, double lon2) {
    double dLat = to_radians(lat2 - lat1);
    double dLon = to_radians(lon2 - lon1);
    lat1 = to_radians(lat1);
    lat2 = to_radians(lat2);
    double a = sin(dLat / 2) * sin(dLat / 2) + sin(dLon / 2) * sin(dLon / 2) * cos(lat1) * cos(lat2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return EARTH_RADIUS_M * c;
}
double calculate_bearing(double lat1_deg, double lon1_deg, double lat2_deg, double lon2_deg) {
    double dLon_rad = to_radians(lon2_deg - lon1_deg);
    double lat1_rad = to_radians(lat1_deg);
    double lat2_rad = to_radians(lat2_deg);
    double y = sin(dLon_rad) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dLon_rad);
    double bearing_rad = atan2(y, x);
    return fmod((to_degrees(bearing_rad) + 360.0), 360.0);
}
void convert_global_to_ned(double home_lat_deg, double home_lon_deg, double current_lat_deg, double current_lon_deg, double& north_m, double& east_m) {
    double distance = calculate_distance_haversine(home_lat_deg, home_lon_deg, current_lat_deg, current_lon_deg);
    double bearing_deg = calculate_bearing(home_lat_deg, home_lon_deg, current_lat_deg, current_lon_deg);
    double bearing_rad = to_radians(bearing_deg);
    north_m = distance * std::cos(bearing_rad);
    east_m = distance * std::sin(bearing_rad);
}
void convert_ned_to_global(double home_lat_deg, double home_lon_deg, double north_m, double east_m, double& target_lat_deg, double& target_lon_deg) {
    double home_lat_rad = to_radians(home_lat_deg);
    double home_lon_rad = to_radians(home_lon_deg);
    double distance = std::sqrt(north_m * north_m + east_m * east_m);
    double bearing_rad = std::atan2(east_m, north_m);
    double target_lat_rad = std::asin(std::sin(home_lat_rad) * std::cos(distance / EARTH_RADIUS_M) + std::cos(home_lat_rad) * std::sin(distance / EARTH_RADIUS_M) * std::cos(bearing_rad));
    double target_lon_rad = home_lon_rad + std::atan2(std::sin(bearing_rad) * std::sin(distance / EARTH_RADIUS_M) * std::cos(home_lat_rad), std::cos(distance / EARTH_RADIUS_M) - std::sin(home_lat_rad) * std::sin(target_lat_rad));
    target_lat_deg = to_degrees(target_lat_rad);
    target_lon_deg = to_degrees(target_lon_rad);
}


class Figure8 : public rclcpp::Node {
public:
    Figure8() : Node("figure8_mission") {
        this->declare_parameter<std::string>("connection_url", "udp://:14540");
        this->declare_parameter<double>("x_amp", 100.0);
        this->declare_parameter<double>("y_amp", 50.0);
        this->declare_parameter<double>("omega", 0.5);
        this->declare_parameter<double>("duration", 0.0);
        this->declare_parameter<double>("height", 10.0);
        this->declare_parameter<double>("speed", 5.0);
        this->declare_parameter<double>("acceptance_radius", 2.0);
        omega_val = this->get_parameter("omega").as_double();
        double calculated_duration = ((2.0 * M_PI) / omega_val) * 2;
        this->set_parameter(rclcpp::Parameter("duration", calculated_duration));
        connection_url = this->get_parameter("connection_url").as_string();
        x_amp = this->get_parameter("x_amp").as_double();
        y_amp = this->get_parameter("y_amp").as_double();
        height = this->get_parameter("height").as_double();
        speed = this->get_parameter("speed").as_double();
        acceptance_radius = this->get_parameter("acceptance_radius").as_double();
        duration = this->get_parameter("duration").as_double();
        RCLCPP_INFO_STREAM(this->get_logger(), "URL: " << connection_url);
        std::stringstream settings_ss;
        settings_ss << "Settings: x_amp: " << std::fixed << std::setprecision(1) << x_amp
                    << ", y_amp: " << std::setprecision(1) << y_amp
                    << ", omega: " << std::setprecision(2) << omega_val
                    << ", duration: " << std::setprecision(1) << duration
                    << ", height: " << std::setprecision(1) << height
                    << ", speed: " << std::setprecision(1) << speed
                    << ", acceptance_radius: " << std::setprecision(1) << acceptance_radius;
        RCLCPP_INFO(this->get_logger(), "%s", settings_ss.str().c_str());
        mission_thread_ = std::thread(&Figure8::run_mission, this);
    }
    ~Figure8() {
        if (mission_thread_.joinable()) {
            mission_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "Figure8 Mission shutdown.");
    }

private:
    enum class MissionState { INITIAL, PRE_FLIGHT_CHECKS, ARMING, TAKING_OFF, UPLOADING_MISSION, EXECUTING_MISSION, LANDING, FINISHED };
    MissionState current_state_ = MissionState::INITIAL;
    std::thread mission_thread_;
    std::string connection_url;
    double x_amp, y_amp, omega_val, duration, height, speed, acceptance_radius;
    std::shared_ptr<mavsdk::Mavsdk> mavsdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::shared_ptr<mavsdk::Action> action_;
    std::shared_ptr<mavsdk::Telemetry> telemetry_;
    std::shared_ptr<mavsdk::Mission> mission_;
    bool initial_health_ok_ = false;
    bool home_position_set_ = false;
    double home_latitude_{0.0};
    double home_longitude_{0.0};
    std::atomic<bool> mission_upload_finished_{false};
    std::atomic<bool> mission_finished_{false};
    std::atomic<mavsdk::Mission::Result> mission_result_{mavsdk::Mission::Result::Unknown};

    // *** 최종 수정된 함수 ***
    mavsdk::Mission::MissionPlan create_figure8_mission_plan() {
        // MissionItem 객체를 직접 담는 벡터를 사용합니다.
        std::vector<mavsdk::Mission::MissionItem> mission_items;
        int num_points = 100;

        for (int i = 0; i <= num_points; ++i) {
            double t = (static_cast<double>(i) / static_cast<double>(num_points)) * duration;
            double north_m = x_amp * std::sin(omega_val * t);
            double east_m = y_amp * std::sin(2.0 * omega_val * t);
            double dx_dt = x_amp * omega_val * std::cos(omega_val * t);
            double dy_dt = y_amp * 2.0 * omega_val * std::cos(2.0 * omega_val * t);
            float yaw_deg = to_degrees(std::atan2(dy_dt, dx_dt));
            double target_lat, target_lon;
            convert_ned_to_global(home_latitude_, home_longitude_, north_m, east_m, target_lat, target_lon);

            // 스마트 포인터 대신 MissionItem 객체를 직접 생성합니다.
            mavsdk::Mission::MissionItem new_item;
            new_item.latitude_deg = target_lat;
            new_item.longitude_deg = target_lon;
            new_item.relative_altitude_m = static_cast<float>(height);
            new_item.speed_m_s = static_cast<float>(speed);
            new_item.acceptance_radius_m = static_cast<float>(acceptance_radius);
            new_item.yaw_deg = yaw_deg;
            new_item.is_fly_through = (i < num_points);
            new_item.loiter_time_s = 0.0f; // fly_through를 위해 loiter time을 0으로 설정

            // 벡터에 객체의 복사본을 추가합니다.
            mission_items.push_back(new_item);
        }

        if (!mission_items.empty()) {
            // 포인터(->)가 아닌 멤버 접근(.) 연산자를 사용합니다.
            mission_items.back().is_fly_through = false;
        }

        mavsdk::Mission::MissionPlan mission_plan;
        // 이제 양쪽의 타입이 일치하여 할당이 가능합니다.
        mission_plan.mission_items = mission_items;
        return mission_plan;
    }

    void run_mission() {
        mavsdk_ = std::make_shared<mavsdk::Mavsdk>(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation});
        mavsdk::ConnectionResult connection_result = mavsdk_->add_any_connection(connection_url);
        if (connection_result != mavsdk::ConnectionResult::Success) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Connection failed");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "MAVSDK connection established.");
        RCLCPP_INFO(this->get_logger(), "Waiting to discover system...");
        auto prom = std::promise<std::shared_ptr<mavsdk::System>>();
        auto fut = prom.get_future();
        mavsdk_->subscribe_on_new_system([&prom, this]() {
            auto system = mavsdk_->systems().back();
            if (system->has_autopilot()) {
                RCLCPP_INFO(this->get_logger(), "Discovered autopilot system.");
                prom.set_value(system);
                mavsdk_->subscribe_on_new_system(nullptr);
            }
        });
        if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "No autopilot system found.");
            rclcpp::shutdown();
            return;
        }
        system_ = fut.get();
        action_ = std::make_shared<mavsdk::Action>(system_);
        telemetry_ = std::make_shared<mavsdk::Telemetry>(system_);
        mission_ = std::make_shared<mavsdk::Mission>(system_);
        telemetry_->subscribe_health_all_ok([this](bool all_ok) {
            if (!initial_health_ok_ && all_ok) {
                RCLCPP_INFO(this->get_logger(), "System health is OK.");
                initial_health_ok_ = true;
            }
        });
        telemetry_->subscribe_home([this](mavsdk::Telemetry::Position home_pos) {
            if (!home_position_set_) {
                home_latitude_ = home_pos.latitude_deg;
                home_longitude_ = home_pos.longitude_deg;
                home_position_set_ = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "Home position set: Lat=" << std::fixed << std::setprecision(7) << home_latitude_ << ", Lon=" << home_longitude_);
            }
        });
        mission_->subscribe_mission_progress([this](mavsdk::Mission::MissionProgress progress) {
            RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Mission progress: " << progress.current << " / " << progress.total);
            if (progress.current >= progress.total && progress.total > 0) {
                mission_finished_ = true;
            }
        });
        current_state_ = MissionState::PRE_FLIGHT_CHECKS;
        while (current_state_ != MissionState::FINISHED && rclcpp::ok()) {
            switch (current_state_) {
                case MissionState::PRE_FLIGHT_CHECKS:
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: PRE_FLIGHT_CHECKS");
                    if (telemetry_->health_all_ok() && home_position_set_) {
                        RCLCPP_INFO(this->get_logger(), "Pre-flight checks passed.");
                        current_state_ = MissionState::ARMING;
                    }
                    break;
                case MissionState::ARMING:
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: ARMING");
                    {
                        const auto arm_result = action_->arm();
                        if (arm_result == mavsdk::Action::Result::Success) {
                            RCLCPP_INFO(this->get_logger(), "Armed successfully.");
                            current_state_ = MissionState::TAKING_OFF;
                        } else {
                            RCLCPP_WARN_STREAM(this->get_logger(), "Arming failed");
                        }
                    }
                    break;
                case MissionState::TAKING_OFF:
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: TAKING_OFF");
                    {
                        action_->set_takeoff_altitude(height);
                        const auto takeoff_result = action_->takeoff();
                        if (takeoff_result == mavsdk::Action::Result::Success) {
                            RCLCPP_INFO(this->get_logger(), "Takeoff command sent. Waiting to reach altitude.");
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            float current_alt = 0.0f;
                            if (telemetry_->health_all_ok()) { current_alt = telemetry_->position().relative_altitude_m; }
                            while (current_alt < height * 0.95f && rclcpp::ok()) {
                                RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Taking off... Current altitude: " << std::fixed << std::setprecision(2) << current_alt << " m");
                                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                                if (telemetry_->health_all_ok()) { current_alt = telemetry_->position().relative_altitude_m; }
                            }
                            RCLCPP_INFO_STREAM(this->get_logger(), "Takeoff complete. Altitude: " << std::fixed << std::setprecision(2) << current_alt << " m");
                            current_state_ = MissionState::UPLOADING_MISSION;
                        } else {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "Takeoff failed");
                            current_state_ = MissionState::LANDING;
                        }
                    }
                    break;
                case MissionState::UPLOADING_MISSION:
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: UPLOADING_MISSION");
                    {
                        mission_upload_finished_ = false;
                        RCLCPP_INFO(this->get_logger(), "Creating mission plan...");
                        auto mission_plan = create_figure8_mission_plan();
                        RCLCPP_INFO_STREAM(this->get_logger(), "Uploading mission with " << mission_plan.mission_items.size() << " items...");
                        mission_->upload_mission_async(mission_plan, [this](mavsdk::Mission::Result result) {
                            mission_result_ = result;
                            mission_upload_finished_ = true;
                        });
                        while (!mission_upload_finished_ && rclcpp::ok()) {
                            std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        }
                        if (mission_result_ == mavsdk::Mission::Result::Success) {
                            RCLCPP_INFO(this->get_logger(), "Mission uploaded successfully.");
                            current_state_ = MissionState::EXECUTING_MISSION;
                        } else {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "Mission upload failed");
                            current_state_ = MissionState::LANDING;
                        }
                    }
                    break;
                case MissionState::EXECUTING_MISSION:
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: EXECUTING_MISSION");
                    {
                        static bool mission_started = false;
                        if (!mission_started) {
                            mission_finished_ = false;
                            mission_->start_mission_async([this](mavsdk::Mission::Result result) {
                                if (result == mavsdk::Mission::Result::Success) {
                                    RCLCPP_INFO(this->get_logger(), "Mission started.");
                                    mission_started = true;
                                } else {
                                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to start mission");
                                    current_state_ = MissionState::LANDING;
                                }
                            });
                        }
                        if (mission_finished_) {
                            RCLCPP_INFO(this->get_logger(), "Mission finished.");
                            current_state_ = MissionState::LANDING;
                        }
                    }
                    break;
                case MissionState::LANDING:
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "State: LANDING");
                    {
                        const auto land_result = action_->land();
                        if (land_result == mavsdk::Action::Result::Success) {
                            RCLCPP_INFO(this->get_logger(), "Landing command sent.");
                            while (telemetry_->in_air() && rclcpp::ok()) {
                                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for landing...");
                                std::this_thread::sleep_for(std::chrono::seconds(1));
                            }
                            RCLCPP_INFO(this->get_logger(), "Landed.");
                            const auto disarm_result = action_->disarm();
                            if (disarm_result == mavsdk::Action::Result::Success) {
                                RCLCPP_INFO(this->get_logger(), "Disarmed.");
                            } else {
                                RCLCPP_ERROR_STREAM(this->get_logger(), "Disarm failed");
                            }
                            current_state_ = MissionState::FINISHED;
                        } else {
                            RCLCPP_ERROR_STREAM(this->get_logger(), "Landing failed");
                            current_state_ = MissionState::FINISHED;
                        }
                    }
                    break;
                default:
                    break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        RCLCPP_INFO(this->get_logger(), "Mission thread finished.");
        rclcpp::shutdown();
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Figure8>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}