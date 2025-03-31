#include "av_hardware/av_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace av_hardware
{
AvHardwareInterface::AvHardwareInterface(){
   
    
}
hardware_interface::CallbackReturn AvHardwareInterface::on_init(const hardware_interface::HardwareInfo & /*info*/)
{
    
};

hardware_interface::CallbackReturn AvHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
};

hardware_interface::CallbackReturn AvHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
};

hardware_interface::CallbackReturn AvHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
};

std::vector<hardware_interface::StateInterface> AvHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // state_interfaces.emplace_back("joint1", "position", &joint1_position_);
    return state_interfaces;
}
std::vector<hardware_interface::CommandInterface> AvHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // command_interfaces.emplace_back("joint1", "position", &joint1_command_);
    return command_interfaces;
}

hardware_interface::return_type AvHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type AvHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    return hardware_interface::return_type::OK;
}

};

PLUGINLIB_EXPORT_CLASS(av_hardware::AvHardwareInterface, hardware_interface::ActuatorInterface)