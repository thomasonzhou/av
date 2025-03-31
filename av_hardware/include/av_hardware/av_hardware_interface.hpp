#ifndef AV_HARDWARE_INTERFACE_HPP_
#define AV_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/actuator_interface.hpp"
#include "cybergear_interface.hpp"
#include <memory>

namespace av_hardware
{
class AvHardwareInterface : public hardware_interface::ActuatorInterface{
public:
    AvHardwareInterface();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
private:
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> command_position_;
    std::vector<double> command_velocity_;
};

};

#endif // AV_HARDWARE_INTERFACE_HPP_