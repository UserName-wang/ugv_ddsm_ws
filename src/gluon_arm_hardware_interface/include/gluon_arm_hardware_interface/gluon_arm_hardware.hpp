#ifndef GLUON_ARM_HARDWARE_HPP
#define GLUON_ARM_HARDWARE_HPP 

#include "hardware_interface/system_interface.hpp"
#include "gluon_arm_hardware_interface/actuator_ne30_sdk.hpp"

namespace gluon_arm_hardware{
class GluonArmHardwareInterface : public hardware_interface::SystemInterface {
public: 
    //lifecycle node override: unconfigure->on_configure, 
    //inactivate->on_activate, active->on_deactivate, inactivate->on_cleanup, cleanup->on_shutdown->on_cleanup
    hardware_interface::callbackReturn
        on_configure(const rclcpp_lifecycle::state & previous_state) override;
    hardware_interface::callbackReturn
        on_active(const rclcpp_lifecycle::state & previous_state) override;
    hardware_interface::callbackReturn
        on_deactive(const rclcpp_lifecycle::state & previous_state) override;

    //system interface override
    hardware_interface::callbackReturn
        on_init(const hardware_interface::HardwareInfo & info) override;   
    hardware_interface::return_type 
        read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type 
        write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    //std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
private: 
    std::shared_ptf<MintascaSDK> driver_;
    int axis_joint_1_ = 1;
    int axis_joint_2_ = 2;
    int axis_joint_3_ = 3;
    int axis_joint_4_ = 4;
    int axis_joint_5_ = 5;
    int axis_joint_6_ = 6;
    std::string ip_address = "192.168.1.30";
    const uint16_t port = 2000;

};
}

#endif// GLUON_ARM_HARDWARE_HPP