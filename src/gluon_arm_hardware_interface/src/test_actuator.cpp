#include "rclcpp/rclcpp.hpp"
#include "gluon_arm_hardware_interface/actuator_ne30_sdk.hpp"
#include <thread>

int main(int argc, char** argv)
{ 
    auto actuators = MintascaSDK();
    actuators.setDebugMode(true);
    if (!actuators.initialize("192.168.1.30", 2000)) 
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to initialize SDK");
        return -1;
    }

    // Enable actuator 5
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enabling actuator 5...");
    if (!actuators.enableActuator(5)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to enable actuator 5");
        return -1;
    }

    // 检查执行器错误状态
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checking error status...");
    uint32_t error_code = actuators.readErrorCode(5);
    if (error_code != 0) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Actuator has error code: 0x%08X", error_code);
        // 尝试清除错误
        if (actuators.clearError(5)) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error cleared successfully");
        } else {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to clear error");
        }
    } else {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No errors detected");
    }

    // Switch actuator 5 to current mode (mode 1)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Switching actuator 5 to current mode...");
    if (!actuators.controlSwitching(5, 1)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to switch actuator 5 to current mode");
        // Disable actuator before returning
        actuators.disableActuator(5);
        return -1;
    }

    // Set target current to 0A for safety
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Setting target current to 0A...");
    // Now we can use the standard writeTargetTorque function since it handles no-response case
    if (!actuators.writeTargetTorque(5, 0.0f)) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Failed to set target current, but continuing for safety");
        // We don't return here to ensure we disable the actuator
    }

    // Read current position of actuator 5
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reading current position of actuator 5...");
    float position = actuators.readCurrentPosition(5);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Actuator 5 position: %f", position);

    // Wait a bit to allow the actuator to run
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Read position again after waiting
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reading position after waiting...");
    position = actuators.readCurrentPosition(5);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Actuator 5 position: %f", position);

    // Disable actuator 5 before exiting
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Disabling actuator 5...");
    if (!actuators.disableActuator(5)) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to disable actuator 5");
        return -1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Actuator 5 successfully disabled. Exiting.");

    return 0;
}