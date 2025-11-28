/////////////////////////////////////////////////////////////////////////////////
// Servo Scanner Tool
// 
// This tool scans the serial bus to find which servo IDs are actually connected
// and responding to queries.
//
// Usage: ros2 run robodog robodog_servo_scanner
/////////////////////////////////////////////////////////////////////////////////
#include "rclcpp/rclcpp.hpp"
#include "Servos/LX16a.h"
#include "io_context/io_context.hpp"

#include <chrono>
#include <iomanip>

using drivers::common::IoContext;

class ServoScannerNode : public rclcpp::Node
{
public:
    ServoScannerNode() : Node("servo_scanner")
    {
        RCLCPP_INFO(this->get_logger(), "Servo Scanner Started");
        
        // Parameters
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("start_id", 1);
        this->declare_parameter("end_id", 20);
        
        std::string port = this->get_parameter("port").as_string();
        int start_id = this->get_parameter("start_id").as_int();
        int end_id = this->get_parameter("end_id").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Port: %s", port.c_str());
        RCLCPP_INFO(this->get_logger(), "  Scan Range: %d to %d", start_id, end_id);
        
        // Initialize IoContext
        auto io_ctx = std::make_shared<IoContext>();
        
        // Initialize LX16a
        auto lx16a = std::make_shared<LX16a>(io_ctx);
        if (!lx16a->open(port)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully\n");
        
        // Run scanner
        scan_servos(lx16a, start_id, end_id);
    }

    void scan_servos(std::shared_ptr<LX16a> lx16a, int start_id, int end_id)
    {
        RCLCPP_INFO(this->get_logger(), "Scanning for connected servos...\n");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "ID | Position | Status");
        RCLCPP_INFO(this->get_logger(), "---|----------|-------");
        
        int found_count = 0;
        
        for (int id = start_id; id <= end_id; ++id) {
            int16_t pos = 0;
            
            bool pos_ok = lx16a->get_pos(id, pos);
            
            if (pos_ok) {
                found_count++;
                RCLCPP_INFO(this->get_logger(), "%2d | %8d | ✓ OK", id, pos);
            } else {
                RCLCPP_INFO(this->get_logger(), "%2d |    N/A   | ✗ No response", id);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));  // Small delay between queries
        }
        
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Found %d responsive servo(s)", found_count);
        RCLCPP_INFO(this->get_logger(), "========================================");
        
        if (found_count == 0) {
            RCLCPP_ERROR(this->get_logger(), "");
            RCLCPP_ERROR(this->get_logger(), "No servos found! Troubleshooting steps:");
            RCLCPP_ERROR(this->get_logger(), "1. Check USB cable connection to serial adapter");
            RCLCPP_ERROR(this->get_logger(), "2. Verify servo power supply is connected");
            RCLCPP_ERROR(this->get_logger(), "3. Try different serial port: -p port:=/dev/ttyACM0");
            RCLCPP_ERROR(this->get_logger(), "4. Check servo TTL connections (TX/RX/GND)");
            RCLCPP_ERROR(this->get_logger(), "5. Verify baud rate (should be 115200 for LX16a)");
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoScannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
