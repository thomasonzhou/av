#ifndef AV_HARDWARE_INTERFACE_HPP_
#define AV_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/actuator_interface.hpp"
#include <memory>
#include <array>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <chrono>
#include <thread>

namespace av_hardware
{
class AvHardwareInterface : public hardware_interface::ActuatorInterface{
public:
    AvHardwareInterface();
    ~AvHardwareInterface();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    static const size_t NUM_JOINTS = 2;
private:
    rclcpp::Logger get_logger() const;
    
    static const int left_motor_id_ = 0x7E; // multiply state by -1.0 to get forward as positive
    static const int right_motor_id_ = 0x7F;
    const std::array<int, NUM_JOINTS> motor_ids_;
    const std::vector<std::string> joint_names_;
    const std::array<std::string, NUM_JOINTS> node_names_;
    
    static const inline std::vector<double> multiplier_{-1.0, 1.0};
    std::vector<double> position_{0.0, 0.0};
    std::vector<double> velocity_{0.0, 0.0};
    std::vector<double> command_position_{0.0, 0.0};
    std::vector<double> command_velocity_{0.0, 0.0};
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr left_torque_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr right_torque_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_zero_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_zero_client_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_joint_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_joint_sub_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_pub_;

    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;
};

};

#endif // AV_HARDWARE_INTERFACE_HPP_