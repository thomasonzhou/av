#include "av_hardware/av_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace av_hardware
{
AvHardwareInterface::AvHardwareInterface():
    ActuatorInterface(), 
    motor_ids_{left_motor_id_, right_motor_id_},
    joint_names_{"left_wheel_joint", "right_wheel_joint"},
    node_names_{"cybergear_left", "cybergear_right"}{
    
    node_ = rclcpp::Node::make_shared("service_client");
    left_torque_client_ = node_->create_client<std_srvs::srv::SetBool>("/"+ node_names_.at(0)+"/enable_torque");
    right_torque_client_ = node_->create_client<std_srvs::srv::SetBool>("/"+ node_names_.at(1)+"/enable_torque");
    
    left_zero_client_ = node_->create_client<std_srvs::srv::Trigger>("/"+ node_names_.at(0)+"/zero_position");
    right_zero_client_ = node_->create_client<std_srvs::srv::Trigger>("/"+ node_names_.at(1)+"/zero_position");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    left_joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/"+ node_names_.at(0)+"/joint_state", qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            position_[0] = multiplier_[0] * msg->position[0];
            velocity_[0] = multiplier_[0] * msg->velocity[0];
        });
    right_joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/"+ node_names_.at(1)+"/joint_state", qos,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            position_[1] = msg->position[0];
            velocity_[1] = msg->velocity[0];
        });
    joint_trajectory_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory", 10);

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
    RCLCPP_INFO(get_logger(), "Initialized motors with IDs: %d, %d", motor_ids_.at(0), motor_ids_.at(1));
}

AvHardwareInterface::~AvHardwareInterface()
{
    executor_->cancel();
    executor_thread_.join();
    RCLCPP_INFO(get_logger(), "Shutting down motors with IDs: %d, %d", motor_ids_.at(0), motor_ids_.at(1));
}

rclcpp::Logger AvHardwareInterface::get_logger() const {
    return rclcpp::get_logger("AvHardwareInterface");
}


hardware_interface::CallbackReturn AvHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
      }
    if (joint_names_.size() != NUM_JOINTS) {
    RCLCPP_FATAL(get_logger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                    NUM_JOINTS);
    return CallbackReturn::ERROR;
    }
    // TODO: add executor to run CyberGear nodes directly

    RCLCPP_INFO(get_logger(), "Successfully connected to robot");
    return CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn AvHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Wait for the service to be available
    while(!left_zero_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(get_logger(), "Waiting for left zero service to be available...");
    }
    while(!right_zero_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(get_logger(), "Waiting for right zero service to be available...");
    }
    auto left_zero_future = left_zero_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    auto right_zero_future = right_zero_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    if (left_zero_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service zero_position for left motor");
        return hardware_interface::CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(get_logger(), "Successfully called service zero_position for left motor");
    }
    
    if (right_zero_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service zero_position for right motor");
        return hardware_interface::CallbackReturn::ERROR;
    }
    else {
        RCLCPP_INFO(get_logger(), "Successfully called service zero_position for right motor");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn AvHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{

    while(!left_torque_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(get_logger(), "Waiting for left torque service to be available...");
    }
    while(!right_torque_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(get_logger(), "Waiting for right torque service to be available...");
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto left_future = left_torque_client_->async_send_request(request);
    auto right_future = right_torque_client_->async_send_request(request);

    if (left_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service enable_torque for left motor");
        return hardware_interface::CallbackReturn::ERROR;
    }
    else{
        RCLCPP_INFO(get_logger(), "Successfully called service enable_torque for left motor");
    }
    if (right_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service enable_torque for right motor");
        return hardware_interface::CallbackReturn::ERROR;
    }
    else{
        RCLCPP_INFO(get_logger(), "Successfully called service enable_torque for right motor");
    }

   return hardware_interface::CallbackReturn::SUCCESS;

};

hardware_interface::CallbackReturn AvHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    auto left_future = left_torque_client_->async_send_request(request);
    auto right_future = right_torque_client_->async_send_request(request);

    if (left_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service disable_torque for left motor");
        return hardware_interface::CallbackReturn::ERROR;
    }
    else{
        RCLCPP_INFO(get_logger(), "Successfully called service disable_torque for left motor");
    }
    if (right_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
    {
        RCLCPP_ERROR(get_logger(), "Failed to call service disable_torque for right motor");
        return hardware_interface::CallbackReturn::ERROR;
    }
    else{
        RCLCPP_INFO(get_logger(), "Successfully called service disable_torque for right motor");
    }

    return hardware_interface::CallbackReturn::SUCCESS;
};

std::vector<hardware_interface::StateInterface> AvHardwareInterface::export_state_interfaces()
{

    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (std::size_t i = 0; i < position_.size(); ++i)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &position_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_[i]));
    }
    return state_interfaces;
}
std::vector<hardware_interface::CommandInterface> AvHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (std::size_t i = 0; i < command_position_.size(); ++i)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &command_position_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &command_velocity_[i]));
    }
    return command_interfaces;
}

hardware_interface::return_type AvHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

    return hardware_interface::return_type::OK;
}
hardware_interface::return_type AvHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    auto send_message = [this](const int i) {
        trajectory_msgs::msg::JointTrajectory msg;
        msg.joint_names = {joint_names_.at(i)};
        msg.points.resize(1);
        msg.points[0].positions.resize(1);
        msg.points[0].velocities.resize(1);
        msg.points[0].accelerations.resize(1);
        msg.points[0].positions[0] = multiplier_.at(i) * command_position_.at(i);
        msg.points[0].velocities[0] = multiplier_.at(i) * command_velocity_.at(i);
        msg.points[0].accelerations[0] = 0.0;
        msg.header.stamp = node_->now();
        joint_trajectory_pub_->publish(msg);
    };

    send_message(0); // left motor
    send_message(1); // right motor

    return hardware_interface::return_type::OK;
}

};

PLUGINLIB_EXPORT_CLASS(av_hardware::AvHardwareInterface, hardware_interface::ActuatorInterface)