#include "flight_node.hpp"
#include <cmath>
#include <algorithm>
using namespace std::chrono_literals;
FlightNode::FlightNode() : Node("global_flight_node") {
    declare_and_load_parameters();
    auto sensor_qos = rclcpp::QoS(10).best_effort();
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", sensor_qos, std::bind(&FlightNode::gps_callback, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&FlightNode::state_callback, this, std::placeholders::_1));
    alt_sub_ = this->create_subscription<std_msgs::msg::Float64>("/mavros/global_position/rel_alt", sensor_qos, std::bind(&FlightNode::alt_callback, this, std::placeholders::_1));
    hdg_sub_ = this->create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg", sensor_qos, std::bind(&FlightNode::hdg_callback, this, std::placeholders::_1));
    camera_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/camera/target", 10, std::bind(&FlightNode::camera_callback, this, std::placeholders::_1));
    cmd_sub_ = this->create_subscription<std_msgs::msg::String>("/drone_command", 10, std::bind(&FlightNode::cmd_callback, this, std::placeholders::_1));
    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    raw_global_pub_ = this->create_publisher<mavros_msgs::msg::GlobalPositionTarget>("/mavros/setpoint_raw/global", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&FlightNode::timer_callback, this));
    last_camera_msg_time_ = this->now();
    last_target_time_ = this->now();
    last_pid_time_ = this->now();
}
void FlightNode::declare_and_load_parameters() {
    this->declare_parameter("pid_yaw.kp", 0.8);
    this->declare_parameter("pid_yaw.ki", 0.0);
    this->declare_parameter("pid_yaw.kd", 0.0);
    this->declare_parameter("pid_forward.kp", 0.8);
    this->declare_parameter("pid_forward.ki", 0.0);
    this->declare_parameter("pid_forward.kd", 0.0);
    this->declare_parameter("desired_distance", 5.0);
    this->declare_parameter("max_forward_speed", 2.0);
    this->declare_parameter("waypoints", std::vector<double>{});
    pid_yaw_.kp = this->get_parameter("pid_yaw.kp").as_double();
    pid_yaw_.ki = this->get_parameter("pid_yaw.ki").as_double();
    pid_yaw_.kd = this->get_parameter("pid_yaw.kd").as_double();
    pid_forward_.kp = this->get_parameter("pid_forward.kp").as_double();
    pid_forward_.ki = this->get_parameter("pid_forward.ki").as_double();
    pid_forward_.kd = this->get_parameter("pid_forward.kd").as_double();
    desired_distance_ = this->get_parameter("desired_distance").as_double();
    max_forward_speed_ = this->get_parameter("max_forward_speed").as_double();
}
void FlightNode::init() {
    drone_ = std::make_shared<DroneInterface>(shared_from_this());
    auto wp_raw = this->get_parameter("waypoints").as_double_array();
    for (size_t i = 0; i + 2 < wp_raw.size(); i += 3) {
        mission_manager_.add_waypoint(wp_raw[i], wp_raw[i+1], wp_raw[i+2]);
    }
}
void FlightNode::cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string cmd = msg->data;
    if (cmd == "enable_detection") detection_enabled_ = true;
    else if (cmd == "disable_detection") detection_enabled_ = false;
    else if (cmd == "enable_patrol") patrol_enabled_ = true;
    else if (cmd == "disable_patrol") patrol_enabled_ = false;
}
void FlightNode::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    gps_ready_ = true; c_lat_ = msg->latitude; c_lon_ = msg->longitude; c_alt_ = msg->altitude;
}
void FlightNode::camera_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    last_camera_msg_time_ = this->now(); target_angle_ = msg->x; target_distance_ = msg->y; target_detected_ = (msg->z > 0.5);
}
void FlightNode::hdg_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    current_yaw_ = (90.0 - msg->data) * M_PI / 180.0;
    while (current_yaw_ > M_PI) current_yaw_ -= 2.0 * M_PI;
    while (current_yaw_ <= -M_PI) current_yaw_ += 2.0 * M_PI;
}
void FlightNode::timer_callback() {
    if (!gps_ready_) return;
    if (current_state_.mode != "GUIDED") { drone_->set_mode("GUIDED"); return; } 
    if (!current_state_.armed) { drone_->arm(true); return; }
    auto current_time = this->now();
    double dt = (current_time - last_pid_time_).seconds();
    last_pid_time_ = current_time;
    if (dt <= 0.0) dt = 0.01;
    if ((current_time - last_camera_msg_time_).seconds() > 1.0) target_detected_ = false;
    if (!detection_enabled_) target_detected_ = false;
    if (flight_state_ == 0) {
        double start_alt = 10.0; 
        if (mission_manager_.get_total_waypoints() > 0) start_alt = mission_manager_.get_current_target().alt;
        static bool takeoff_initiated = false;
        if (!takeoff_initiated) { drone_->takeoff(start_alt); takeoff_initiated = true; return; }
        if (c_rel_alt_ > (start_alt - 1.0)) flight_state_ = 1;
        return;
    }
    if (target_detected_) { flight_state_ = 2; last_target_time_ = current_time; }
    else if (flight_state_ == 2) { flight_state_ = 3; }
    if (flight_state_ == 3) { if ((current_time - last_target_time_).seconds() >= 10.0) flight_state_ = 1; }
    if (flight_state_ == 2) { 
        geometry_msgs::msg::TwistStamped vel_msg;
        vel_msg.header.stamp = current_time; vel_msg.header.frame_id = "map"; 
        double yaw_speed = pid_yaw_.calculate(-target_angle_, dt);
        double fwd_speed = std::clamp(pid_forward_.calculate(target_distance_ - desired_distance_, dt), -max_forward_speed_, max_forward_speed_);
        vel_msg.twist.linear.x = fwd_speed * std::cos(current_yaw_);
        vel_msg.twist.linear.y = fwd_speed * std::sin(current_yaw_);
        vel_msg.twist.angular.z = yaw_speed;
        vel_pub_->publish(vel_msg);
    } else if (flight_state_ == 1) {
        if (!patrol_enabled_) {
            geometry_msgs::msg::TwistStamped vel_msg;
            vel_msg.header.stamp = current_time; vel_msg.header.frame_id = "base_link";
            vel_pub_->publish(vel_msg); return;
        }
        if (!mission_manager_.is_mission_finished()) {
            Waypoint wp = mission_manager_.get_current_target();
            if (mission_manager_.is_at_destination(c_lat_, c_lon_, 2.0)) mission_manager_.next_waypoint();
            else {
                mavros_msgs::msg::GlobalPositionTarget pos_msg;
                pos_msg.header.stamp = current_time; pos_msg.coordinate_frame = 6; pos_msg.type_mask = 4088;
                pos_msg.latitude = wp.lat; pos_msg.longitude = wp.lon; pos_msg.altitude = (c_alt_ - c_rel_alt_) + wp.alt; 
                pos_msg.yaw = (M_PI / 2.0) - mission_manager_.calculate_bearing(c_lat_, c_lon_, wp.lat, wp.lon);
                raw_global_pub_->publish(pos_msg);
            }
        } else mission_manager_.reset();
    } else if (flight_state_ == 3) {
        geometry_msgs::msg::TwistStamped vel_msg;
        vel_msg.header.stamp = current_time; vel_msg.header.frame_id = "base_link";
        vel_pub_->publish(vel_msg);
    }
}
void FlightNode::state_callback(const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; }
void FlightNode::alt_callback(const std_msgs::msg::Float64::SharedPtr msg) { c_rel_alt_ = msg->data; }
