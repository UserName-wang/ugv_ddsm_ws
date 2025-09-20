#include "rclcpp/rclcpp.hpp"
#include "gluon_arm_hardware_interface/actuator_ne30_sdk.hpp"
#include <thread>
#include <vector>

int main(int argc, char** argv)
{ 
    auto actuators = MintascaSDK();
    actuators.setDebugMode(true);
    if (!actuators.initialize("192.168.1.30", 2000)) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize SDK");
        return -1;
    }

    // Get all discovered actuators
    auto actuator_info = actuators.getActuatorInfo();
    std::vector<uint8_t> actuator_ids;
    for (const auto& pair : actuator_info) {
        actuator_ids.push_back(pair.first);
    }

    if (actuator_ids.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No actuators found");
        return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Found %zu actuators", actuator_ids.size());

    // Enable all actuators
    for (uint8_t id : actuator_ids) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enabling actuator %d...", id);
        if (!actuators.enableActuator(id)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to enable actuator %d", id);
            return -1;
        }
    }

    // Check error status for all actuators
    for (uint8_t id : actuator_ids) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checking error status for actuator %d...", id);
        uint32_t error_code = actuators.readErrorCode(id);
        if (error_code != 0) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Actuator %d has error code: 0x%08X", id, error_code);
            // Try to clear error
            if (actuators.clearError(id)) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error cleared successfully for actuator %d", id);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to clear error for actuator %d", id);
            }
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No errors detected for actuator %d", id);
        }
    }

    // Switch all actuators to current mode (mode 1)
    for (uint8_t id : actuator_ids) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Switching actuator %d to current mode...", id);
        if (!actuators.controlSwitching(id, 1)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to switch actuator %d to current mode", id);
            // Disable previously enabled actuators before returning
            for (uint8_t enabled_id : actuator_ids) {
                if (enabled_id <= id) {
                    actuators.disableActuator(enabled_id);
                }
            }
            return -1;
        }
    }

    // Set target current to 0A for safety for all actuators
    for (uint8_t id : actuator_ids) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting target current to 0A for actuator %d...", id);
        // Use the standard writeTargetTorque function since it handles no-response case
        if (!actuators.writeTargetTorque(id, 0.0f)) {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to set target current for actuator %d, but continuing for safety", id);
            // We don't return here to ensure we disable all actuators
        }
    }

    // Read current position of all actuators
    for (uint8_t id : actuator_ids) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reading current position of actuator %d...", id);
        float position = actuators.readCurrentPosition(id);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Actuator %d position: %f", id, position);
    }

    // Wait a bit to allow the actuators to run
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Read position again after waiting for all actuators
    for (uint8_t id : actuator_ids) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reading position of actuator %d after waiting...", id);
        float position = actuators.readCurrentPosition(id);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Actuator %d position: %f", id, position);
    }

    // Disable all actuators before exiting
    bool all_disabled = true;
    for (uint8_t id : actuator_ids) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disabling actuator %d...", id);
        if (!actuators.disableActuator(id)) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disable actuator %d", id);
            all_disabled = false;
        }
    }
    
    if (all_disabled) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "All actuators successfully disabled. Exiting.");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disable one or more actuators. Exiting.");
        return -1;
    }

    return 0;
}