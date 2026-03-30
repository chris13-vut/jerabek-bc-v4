#include "drone_interface.hpp"

using namespace std::chrono_literals;

DroneInterface::DroneInterface(rclcpp::Node::SharedPtr node) : node_(node) {
    mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    arm_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    takeoff_client_ = node_->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    land_client_ = node_->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
}

bool DroneInterface::set_mode(std::string mode) {
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    if (!mode_client_->wait_for_service(1s)) return false;
    mode_client_->async_send_request(req); 
    return true; 
}

bool DroneInterface::arm(bool value) {
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = value;
    if (!arm_client_->wait_for_service(1s)) return false;
    arm_client_->async_send_request(req);
    return true;
}

bool DroneInterface::takeoff(double altitude) {
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->altitude = altitude;
    if (!takeoff_client_->wait_for_service(1s)) return false;
    takeoff_client_->async_send_request(req);
    return true;
}

bool DroneInterface::land() {
    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    if (!land_client_->wait_for_service(1s)) return false;
    land_client_->async_send_request(req);
    return true;
}
