#ifndef GLUON_ARM_HARDWARE_HPP
#define GLUON_ARM_HARDWARE_HPP 

#include "hardware_interface/system_interface.hpp"
#include "gluon_arm_hardware_interface/actuator_ne30_sdk.hpp"
#include <vector>
#include <map>
#include <pluginlib/class_list_macros.hpp>

namespace gluon_arm_hardware{
class GluonArmHardwareInterface : public hardware_interface::SystemInterface {
public: 
    //lifecycle node override: unconfigure->on_configure, 
    //inactivate->on_activate, active->on_deactivate, inactivate->on_cleanup, cleanup->on_shutdown->on_cleanup
    hardware_interface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    //system interface override
    hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;   
    hardware_interface::return_type 
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type 
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private: 
    std::shared_ptr<MintascaSDK> driver_;
    // Map joint names to actuator IDs
    std::map<std::string, uint8_t> joint_to_actuator_;
    std::string ip_address = "192.168.1.30";
    const uint16_t port = 2000;
    
    // State interfaces for joint positions
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;
    
    // Command interfaces for joint positions
    std::vector<double> hw_commands_;

};
}

PLUGINLIB_EXPORT_CLASS(gluon_arm_hardware::GluonArmHardwareInterface, hardware_interface::SystemInterface)

#endif// GLUON_ARM_HARDWARE_HPP