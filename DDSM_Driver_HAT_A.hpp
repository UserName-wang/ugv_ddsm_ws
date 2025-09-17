#ifndef DDSM_DRIVER_HAT_A_HPP
#define DDSM_DRIVER_HAT_A_HPP

#include <string>
#include <iostream>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>

class DDSMDriverHAT {
private:
    std::string ipAddress;
    
    // Callback function to handle HTTP response
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* response) {
        size_t totalSize = size * nmemb;
        response->append((char*)contents, totalSize);
        return totalSize;
    }

public:
    /**
     * Constructor
     * @param ip IP address of the DDSM Driver HAT
     */
    DDSMDriverHAT(const std::string& ip) : ipAddress(ip) {
        curl_global_init(CURL_GLOBAL_DEFAULT);
    }
    
    /**
     * Initialization function
     * Sets up heartbeat, stops all motors, and checks if all four motors are online
     * @return true if successful, false if any motor is offline
     */
    bool initialize() {
        // Set heartbeat function with default time (2000ms)
        if (!setHeartbeat()) {
            std::cerr << "Failed to set heartbeat function" << std::endl;
            return false;
        }
        
        // Stop all four motors (ID 1-4) with default acceleration (3)
        for (int id = 1; id <= 4; id++) {
            if (!speedControl(id, 0)) {
                std::cerr << "Failed to stop motor " << id << std::endl;
                return false;
            }
        }
        
        // Check if all four motors are online
        for (int id = 1; id <= 4; id++) {
            if (!getMotorInfo(id)) {
                std::cerr << "Motor " << id << " is offline" << std::endl;
                return false;
            }
        }
        
        return true;
    }
    
    /**
     * Destructor
     */
    ~DDSMDriverHAT() {
        curl_global_cleanup();
    }
    
    /**
     * 1. Set motor ID
     * Example JSON command: {"T":10011,"id":1}
     * @param id Motor ID (1-253)
     */
    bool setMotorID(int id) {
        Json::Value command;
        command["T"] = 10011;
        command["id"] = id;
        
        return sendCommand(command);
    }
    
    /**
     * 2. Set heartbeat function
     * Example JSON command: {"T":11001,"time":2000}
     * @param time Time in milliseconds (-1 to disable, >0 to enable, default 2000)
     */
    bool setHeartbeat(int time = 2000) {
        Json::Value command;
        command["T"] = 11001;
        command["time"] = time;
        
        return sendCommand(command);
    }
    
    /**
     * 3. Speed control
     * Example JSON command: {"T":10010,"id":1,"cmd":50,"act":3}
     * @param id Motor ID
     * @param cmd Command value (speed in rpm for DDSM115, 0.1rpm for DDSM210)
     * @param act Acceleration time (0.1ms per unit, default 3)
     */
    bool speedControl(int id, int cmd, int act = 3) {
        Json::Value command;
        command["T"] = 10010;
        command["id"] = id;
        command["cmd"] = cmd;
        command["act"] = act;
        
        return sendCommand(command);
    }
    
    /**
     * 4. Motor mode switching
     * Example JSON command: {"T":10012,"id":1,"mode":2}
     * @param id Motor ID
     * @param mode Motor mode (0: open loop, 1: current loop, 2: speed loop, 3: position loop)
     */
    bool switchMode(int id, int mode) {
        Json::Value command;
        command["T"] = 10012;
        command["id"] = id;
        command["mode"] = mode;
        
        return sendCommand(command);
    }
    
    /**
     * 5. WiFi settings
     * Various WiFi configuration commands
     */
    bool setWiFiMode(int cmd) {
        Json::Value command;
        command["T"] = 10401;
        command["cmd"] = cmd;
        
        return sendCommand(command);
    }
    
    bool setAPMode(const std::string& ssid, const std::string& password) {
        Json::Value command;
        command["T"] = 10402;
        command["ssid"] = ssid;
        command["password"] = password;
        
        return sendCommand(command);
    }
    
    bool setSTAMode(const std::string& ssid, const std::string& password) {
        Json::Value command;
        command["T"] = 10403;
        command["ssid"] = ssid;
        command["password"] = password;
        
        return sendCommand(command);
    }
    
    bool setAPSTAMode(const std::string& ap_ssid, const std::string& ap_password,
                      const std::string& sta_ssid, const std::string& sta_password) {
        Json::Value command;
        command["T"] = 10404;
        command["ap_ssid"] = ap_ssid;
        command["ap_password"] = ap_password;
        command["sta_ssid"] = sta_ssid;
        command["sta_password"] = sta_password;
        
        return sendCommand(command);
    }
    
    bool getWiFiInfo() {
        Json::Value command;
        command["T"] = 10405;
        
        return sendCommand(command);
    }
    
    bool disconnectWiFi() {
        Json::Value command;
        command["T"] = 10408;
        
        return sendCommand(command);
    }
    
    /**
     * 6. ESP32 settings
     */
    bool restartESP32() {
        Json::Value command;
        command["T"] = 600;
        
        return sendCommand(command);
    }
    
    bool getFlashSpace() {
        Json::Value command;
        command["T"] = 601;
        
        return sendCommand(command);
    }
    
    bool clearNVS() {
        Json::Value command;
        command["T"] = 604;
        
        return sendCommand(command);
    }
    
    /**
     * 7. Get motor feedback
     */
    bool getMotorInfo(int id) {
        Json::Value command;
        command["T"] = 10032;
        command["id"] = id;
        
        return sendCommand(command);
    }
    
    bool getMotorID() {
        Json::Value command;
        command["T"] = 10031;
        
        return sendCommand(command);
    }

private:
    /**
     * Send JSON command to the DDSM Driver HAT
     * @param command JSON command to send
     * @return true if successful, false otherwise
     */
    bool sendCommand(const Json::Value& command) {
        CURL* curl = curl_easy_init();
        if (!curl) {
            std::cerr << "Failed to initialize CURL" << std::endl;
            return false;
        }
        
        // Convert JSON command to string
        Json::StreamWriterBuilder builder;
        builder["indentation"] = ""; // Compact output
        std::string jsonCommand = Json::writeString(builder, command);
        
        std::string url = "http://" + ipAddress + "/command";
        std::string response;
        
        // Set CURL options
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, jsonCommand.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L); // 5 second timeout
        
        // Perform the request
        CURLcode res = curl_easy_perform(curl);
        
        if (res != CURLE_OK) {
            std::cerr << "CURL request failed: " << curl_easy_strerror(res) << std::endl;
            curl_easy_cleanup(curl);
            return false;
        }
        
        long responseCode;
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &responseCode);
        curl_easy_cleanup(curl);
        
        if (responseCode != 200) {
            std::cerr << "HTTP error: " << responseCode << std::endl;
            return false;
        }
        
        return true;
    }
};

#endif // DDSM_DRIVER_HAT_A_HPP