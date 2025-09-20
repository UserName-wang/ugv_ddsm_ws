#include "gluon_arm_hardware_interface/gluon_arm_hardware.hpp"

namespace gluon_arm_hardware{
hardware_interface::callbackReturn GluonArmHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SystemInterface::on_init(info)!=
        hardware_interface::callbackReturn::SUCCESS){
            return hardware_interface::callbackReturn::ERROR;
        }
        info_ = info;
        int axis_joint_1_id = 1;
        int axis_joint_2_id = 2;
        int axis_joint_3_id = 3;
        int axis_joint_4_id = 4;
        int axis_joint_5_id = 5;
        int axis_joint_6_id = 6;
        std::string ip_address = "192.168.1.30";
        const uint16_t port = 2000;
        driver_ = std::make_shared<MintascaSDK>(ip_address, port);
        return hardware_interface::callbackReturn::SUCCESS;
    };
hardware_interface::callbackReturn GluonArmHardwareInterface::on_configure
    (const rclcpp_lifecycle::state & previous_state){
        (void)previous_state;
        if (!driver_->initialize()){
            return hardware_interface::callbackReturn::ERROR;
    };
    return hardware_interface::callbackReturn::SUCCESS;

hardware_interface::callbackReturn GluonArmHardwareInterface::on_active
    (const rclcpp_lifecycle::state & previous_state) {
        (void)previous_state;
        driver_->start();
    };
hardware_interface::callbackReturn GluonArmHardwareInterface::on_deactive
    (const rclcpp_lifecycle::state & previous_state) {

    };

    //system interface override
   
hardware_interface::return_type GluonArmHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period) {

    };
hardware_interface::return_type GluonArmHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period) {

    };
    
}