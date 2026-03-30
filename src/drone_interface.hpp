#ifndef DRONE_INTERFACE_HPP_
#define DRONE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/command_tol.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

class DroneInterface {
public:
     DroneInterface(rclcpp::Node::SharedPtr node);
     bool set_mode(std::string mode);
     bool arm(bool value);
     bool takeoff(double altitude);
     bool land();

private:
     rclcpp::Node::SharedPtr node_;
     rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
     rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
     rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;
     rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_client_;
};

#endif
