#include "DDSM_Driver_HAT_A.hpp"
#include <iostream>
#include <unistd.h>

int main() {
    // Create driver instance with IP address
    // Replace with actual IP address of your DDSM Driver HAT
    DDSMDriverHAT driver("192.168.4.1");
    
    std::cout << "Initializing DDSM motors..." << std::endl;
    
    // Test initialization
    if (!driver.initialize()) {
        std::cerr << "Failed to initialize DDSM motors" << std::endl;
        return -1;
    }
    
    std::cout << "DDSM motors initialized successfully" << std::endl;
    
    // Test setting motor ID
    std::cout << "Setting motor ID to 1..." << std::endl;
    if (!driver.setMotorID(1)) {
        std::cerr << "Failed to set motor ID" << std::endl;
        return -1;
    }
    
    // Test heartbeat function with default value
    std::cout << "Setting heartbeat with default time..." << std::endl;
    if (!driver.setHeartbeat()) {
        std::cerr << "Failed to set heartbeat" << std::endl;
        return -1;
    }
    
    // Test speed control with default acceleration
    std::cout << "Controlling motor speed with default acceleration..." << std::endl;
    if (!driver.speedControl(1, 50)) {
        std::cerr << "Failed to control motor speed" << std::endl;
        return -1;
    }
    
    // Wait for a moment to observe motor movement
    sleep(2);
    
    // Stop the motor
    std::cout << "Stopping motor..." << std::endl;
    if (!driver.speedControl(1, 0)) {
        std::cerr << "Failed to stop motor" << std::endl;
        return -1;
    }
    
    // Test mode switching
    std::cout << "Switching motor mode to speed loop..." << std::endl;
    if (!driver.switchMode(1, 2)) {
        std::cerr << "Failed to switch motor mode" << std::endl;
        return -1;
    }
    
    // Test getting motor info
    std::cout << "Getting motor info..." << std::endl;
    if (!driver.getMotorInfo(1)) {
        std::cerr << "Failed to get motor info" << std::endl;
        return -1;
    }
    
    std::cout << "All tests passed successfully!" << std::endl;
    
    return 0;
}