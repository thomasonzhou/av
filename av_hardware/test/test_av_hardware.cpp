#include <gmock/gmock.h>
#include <string>
#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestAvHardware : public ::testing::Test
{
protected:
  void SetUp() override
  {
    av_hardware_file_ = 
    R"(
    <ros2_control name="GazeboSystem" type="system">
        <hardware>
            <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </hardware>
        <joint name="left_wheel_joint">
            <command_interface name="velocity">
                <param name="min">-10</param>
                <param name="max">10</param>
            </command_interface>
            <state_interface name="position" />
            <state_interface name="velocity" />
        </joint>
        <joint name="right_wheel_joint">
            <command_interface name="velocity">
                <param name="min">-10</param>
                <param name="max">10</param>
            </command_interface>
            <state_interface name="velocity" />
            <state_interface name="position" />
        </joint>
        <joint name="left_caster_wheel_joint">
            <state_interface name="position" />
        </joint>
        <joint name="right_caster_wheel_joint">
            <state_interface name="position" />
        </joint>
    </ros2_control>)";
  }

  std::string av_hardware_file_;
};

TEST_F(TestAvHardware, TestAvHardwareInterface)
{
  auto urdf = ros2_control_test_assets::urdf_head + av_hardware_file_ + ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
