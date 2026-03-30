#ifndef FLIGHT_NODE_HPP_
#define FLIGHT_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/global_position_target.hpp" // PRIDANO: RAW MAVLink struktura
#include "drone_interface.hpp"
#include "mission_manager.hpp"

struct PIDController {
    double kp = 0.0, ki = 0.0, kd = 0.0;
    double integral = 0.0, prev_error = 0.0;

    double calculate(double error, double dt) {
        integral += error * dt;
        integral = std::clamp(integral, -2.0, 2.0); 
        double derivative = (dt > 0.0) ? (error - prev_error) / dt : 0.0;
        prev_error = error;
        return (kp * error) + (ki * integral) + (kd * derivative);
    }
    void reset() { integral = 0.0; prev_error = 0.0; }
};

class FlightNode : public rclcpp::Node {
public:
    FlightNode();
    void init();

private:
    PIDController pid_yaw_;
    PIDController pid_forward_;
    
    double desired_distance_{5.0};     
    double max_forward_speed_{2.0};    

    int flight_state_{0}; 
    rclcpp::Time last_target_time_;
    rclcpp::Time last_camera_msg_time_;
    rclcpp::Time last_pid_time_;
    MissionManager mission_manager_;

    void declare_and_load_parameters();
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
    void alt_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void camera_callback(const geometry_msgs::msg::Point::SharedPtr msg);
    void timer_callback();

    std::shared_ptr<DroneInterface> drone_;
    
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr alt_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr camera_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    
    // ZMENENO: Nyní publikujeme nativní surovou zprávu
    rclcpp::Publisher<mavros_msgs::msg::GlobalPositionTarget>::SharedPtr raw_global_pub_; 

    rclcpp::TimerBase::SharedPtr timer_;

    double c_lat_{0}, c_lon_{0}, c_alt_{0}, c_rel_alt_{0}; 
    double current_yaw_{0.0}; 
    bool gps_ready_{false};
    bool target_detected_{false};
    double target_angle_{0.0};
    double target_distance_{0.0};
    
    mavros_msgs::msg::State current_state_;
};

#endif
