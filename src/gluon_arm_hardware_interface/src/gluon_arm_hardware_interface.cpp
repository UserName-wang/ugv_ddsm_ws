#include "gluon_arm_hardware_interface/gluon_arm_hardware.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gluon_arm_hardware{
hardware_interface::CallbackReturn GluonArmHardwareInterface::on_init
    (const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SystemInterface::on_init(info)!=
        hardware_interface::CallbackReturn::SUCCESS){
            RCLCPP_ERROR(rclcpp::get_logger("gluon_arm_hardware"), "Failed to initialize system interface");
            return hardware_interface::CallbackReturn::ERROR;
        }
        info_ = info;
        
        // Resize vectors to match number of joints
        hw_positions_.resize(info_.joints.size(), 0.0);
        hw_velocities_.resize(info_.joints.size(), 0.0);
        hw_efforts_.resize(info_.joints.size(), 0.0);
        hw_commands_.resize(info_.joints.size(), 0.0);
        
        // Map joint names to actuator IDs (1-6)
        joint_to_actuator_.clear();
        for (size_t i = 0; i < info_.joints.size(); i++) {
            // Map axis_joint_N to actuator ID N
            std::string joint_name = info_.joints[i].name;
            if (joint_name.substr(0, 10) == "axis_joint") {
                uint8_t actuator_id = static_cast<uint8_t>(std::stoi(joint_name.substr(11)));
                joint_to_actuator_[joint_name] = actuator_id;
                RCLCPP_INFO(rclcpp::get_logger("gluon_arm_hardware"), 
                    "Mapped joint '%s' to actuator ID %d", joint_name.c_str(), actuator_id);
            }
        }
        
        // Initialize driver
        driver_ = std::make_shared<MintascaSDK>();
        driver_->setDebugMode(true);
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

std::vector<hardware_interface::StateInterface> GluonArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "position", &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "velocity", &hw_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, "effort", &hw_efforts_[i]));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GluonArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, "position", &hw_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn GluonArmHardwareInterface::on_configure
    (const rclcpp_lifecycle::State & previous_state){
        (void)previous_state;
        if (!driver_->initialize("192.168.1.30", 2000)){
            RCLCPP_ERROR(rclcpp::get_logger("gluon_arm_hardware"), "Failed to initialize SDK");
            return hardware_interface::CallbackReturn::ERROR;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

hardware_interface::CallbackReturn GluonArmHardwareInterface::on_activate
    (const rclcpp_lifecycle::State & previous_state) {
        (void)previous_state;
        //查找在线的执行器，并将其使能，切换到电流模式（电流为0）
        if (!driver_->discoverActuators()) {
            RCLCPP_ERROR(rclcpp::get_logger("gluon_arm_hardware"), "Failed to discover actuators");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        // Get all discovered actuators
        auto actuator_info = driver_->getActuatorInfo();
        
        if (actuator_info.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("gluon_arm_hardware"), "No actuators found");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("gluon_arm_hardware"), "Found %zu actuators", actuator_info.size());

        // Enable all actuators
        for (const auto& pair : actuator_info) {
            uint8_t id = pair.first;
            RCLCPP_INFO(rclcpp::get_logger("gluon_arm_hardware"), "Enabling actuator %d...", id);
            if (!driver_->enableActuator(id)) {
                RCLCPP_ERROR(rclcpp::get_logger("gluon_arm_hardware"), "Failed to enable actuator %d", id);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Check error status for all actuators
        for (const auto& pair : actuator_info) {
            uint8_t id = pair.first;
            RCLCPP_INFO(rclcpp::get_logger("gluon_arm_hardware"), "Checking error status for actuator %d...", id);
            uint32_t error_code = driver_->readErrorCode(id);
            if (error_code != 0) {
                RCLCPP_WARN(rclcpp::get_logger("gluon_arm_hardware"), "Actuator %d has error code: 0x%08X", id, error_code);
                // Try to clear error
                if (driver_->clearError(id)) {
                    RCLCPP_INFO(rclcpp::get_logger("gluon_arm_hardware"), "Error cleared successfully for actuator %d", id);
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("gluon_arm_hardware"), "Failed to clear error for actuator %d", id);
                }
            } else {
                RCLCPP_INFO(rclcpp::get_logger("gluon_arm_hardware"), "No errors detected for actuator %d", id);
            }
        }

        // Switch all actuators to current mode (mode 1) for safety
        for (const auto& pair : actuator_info) {
            uint8_t id = pair.first;
            RCLCPP_INFO(rclcpp::get_logger("gluon_arm_hardware"), "Switching actuator %d to current mode...", id);
            if (!driver_->controlSwitching(id, 1)) {
                RCLCPP_ERROR(rclcpp::get_logger("gluon_arm_hardware"), "Failed to switch actuator %d to current mode", id);
                // Disable previously enabled actuators before returning
                for (const auto& enabled_pair : actuator_info) {
                    uint8_t enabled_id = enabled_pair.first;
                    if (enabled_id <= id) {
                        driver_->disableActuator(enabled_id);
                    }
                }
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Set target current to 0A for safety for all actuators
        for (const auto& pair : actuator_info) {
            uint8_t id = pair.first;
            RCLCPP_INFO(rclcpp::get_logger("gluon_arm_hardware"), "Setting target current to 0A for actuator %d...", id);
            // Use the standard writeTargetTorque function since it handles no-response case
            if (!driver_->writeTargetTorque(id, 0.0f)) {
                RCLCPP_WARN(rclcpp::get_logger("gluon_arm_hardware"), "Failed to set target current for actuator %d, but continuing for safety", id);
                // We don't return here to ensure we disable all actuators
            }
        }
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

hardware_interface::CallbackReturn GluonArmHardwareInterface::on_deactivate
    (const rclcpp_lifecycle::State & previous_state) {
        (void)previous_state;
        
        // Disable all actuators
        if (driver_) {
            auto actuator_info = driver_->getActuatorInfo();
            bool all_disabled = true;
            for (const auto& pair : actuator_info) {
                uint8_t id = pair.first;
                RCLCPP_INFO(rclcpp::get_logger("gluon_arm_hardware"), "Disabling actuator %d...", id);
                if (!driver_->disableActuator(id)) {
                    RCLCPP_ERROR(rclcpp::get_logger("gluon_arm_hardware"), "Failed to disable actuator %d", id);
                    all_disabled = false;
                }
            }
            
            if (!all_disabled) {
                RCLCPP_ERROR(rclcpp::get_logger("gluon_arm_hardware"), "Failed to disable one or more actuators");
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            RCLCPP_INFO(rclcpp::get_logger("gluon_arm_hardware"), "All actuators successfully disabled");
        }
        
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    //system interface override
   
hardware_interface::return_type GluonArmHardwareInterface::read
    (const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void)time;
        (void)period;
        // Read current position of all actuators
        if (driver_) {
            auto actuator_info = driver_->getActuatorInfo();
            for (size_t i = 0; i < info_.joints.size() && i < hw_positions_.size(); ++i) {
                std::string joint_name = info_.joints[i].name;
                if (joint_to_actuator_.find(joint_name) != joint_to_actuator_.end()) {
                    uint8_t actuator_id = joint_to_actuator_[joint_name];
                    // Check if this actuator is available
                    if (actuator_info.find(actuator_id) != actuator_info.end()) {
                        float position_R = driver_->readCurrentPosition(actuator_id);
                        // Convert from R unit to radians
                        // 1 R = 10 degrees, and 1 degree = π/180 radians
                        // So 1 R = 10 * π/180 = π/18 radians
                        double position_rad = position_R * M_PI / 18.0;
                        hw_positions_[i] = position_rad;
                        RCLCPP_DEBUG(rclcpp::get_logger("gluon_arm_hardware"), 
                            "Joint %s (actuator %d) position: %f R (%f rad)", 
                            joint_name.c_str(), actuator_id, position_R, position_rad);
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("gluon_arm_hardware"), 
                            "Actuator %d for joint %s not found", actuator_id, joint_name.c_str());
                    }
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("gluon_arm_hardware"), 
                        "No actuator mapping found for joint %s", joint_name.c_str());
                }
            }
        }
        
        return hardware_interface::return_type::OK;
    }

hardware_interface::return_type GluonArmHardwareInterface::write
    (const rclcpp::Time & time, const rclcpp::Duration & period) {
        (void)time;
        (void)period;
        // For safety, do not execute any position commands
        // All actuators are in current mode with 0A current
        // When enabling position control, convert radians to R units:
        // 1 radian = 18/π degrees = 18/π * 0.1 R = 1.8/π R
        RCLCPP_DEBUG(rclcpp::get_logger("gluon_arm_hardware"), 
            "In safety mode - not executing position commands. All actuators in current mode with 0A current.");
        
        /*
        if (driver_) {
            auto actuator_info = driver_->getActuatorInfo();
            for (size_t i = 0; i < info_.joints.size() && i < hw_commands_.size(); ++i) {
                std::string joint_name = info_.joints[i].name;
                if (joint_to_actuator_.find(joint_name) != joint_to_actuator_.end()) {
                    uint8_t actuator_id = joint_to_actuator_[joint_name];
                    // Check if this actuator is available
                    if (actuator_info.find(actuator_id) != actuator_info.end()) {
                        // Convert from radians to R unit
                        // 1 radian = 18/π degrees = 18/π * 0.1 R = 1.8/π R
                        float target_position_R = static_cast<float>(hw_commands_[i] * 1.8 / M_PI);
                        
                        RCLCPP_DEBUG(rclcpp::get_logger("gluon_arm_hardware"), 
                            "Setting joint %s (actuator %d) to position %f rad (%f R)", 
                            joint_name.c_str(), actuator_id, hw_commands_[i], target_position_R);
                        
                        // Send command to set the position
                        if (!driver_->writeTargetPosition(actuator_id, target_position_R)) {
                            RCLCPP_WARN(rclcpp::get_logger("gluon_arm_hardware"), 
                                "Failed to set position for joint %s (actuator %d)", 
                                joint_name.c_str(), actuator_id);
                        }
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("gluon_arm_hardware"), 
                            "Actuator %d for joint %s not found", actuator_id, joint_name.c_str());
                    }
                } else {
                    RCLCPP_WARN(rclcpp::get_logger("gluon_arm_hardware"), 
                        "No actuator mapping found for joint %s", joint_name.c_str());
                }
            }
        }
        */
        
        return hardware_interface::return_type::OK;
    }
    
}