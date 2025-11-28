/////////////////////////////////////////////////////////////////////////////////
// Servo Controller Hz Diagnostic Tool
// 
// This tool measures the actual update frequency (Hz) of the servo controller
// by querying motor positions repeatedly and measuring the time between updates.
//
// Usage: ros2 run robodog robodog_servo_diagnostics
/////////////////////////////////////////////////////////////////////////////////
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "Factory.h"
#include "Servos/ServoController.h"
#include "Servos/LX16a.h"
#include "io_context/io_context.hpp"

#include <chrono>
#include <vector>
#include <numeric>
#include <iomanip>

using drivers::common::IoContext;

class ServoControllerDiagnosticsNode : public rclcpp::Node
{
public:
    ServoControllerDiagnosticsNode() : Node("servo_controller_diagnostics")
    {
        RCLCPP_INFO(this->get_logger(), "Servo Controller Hz Diagnostic Tool Started");
        
        // Parameters
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("measurement_duration", 10.0);  // seconds
        this->declare_parameter("servo_id", 1);  // Which servo to monitor
        
        std::string port = this->get_parameter("port").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        double measurement_duration = this->get_parameter("measurement_duration").as_double();
        int servo_id = this->get_parameter("servo_id").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Configuration:");
        RCLCPP_INFO(this->get_logger(), "  Port: %s", port.c_str());
        RCLCPP_INFO(this->get_logger(), "  Baud Rate: %d", baud_rate);
        RCLCPP_INFO(this->get_logger(), "  Measurement Duration: %.1f seconds", measurement_duration);
        RCLCPP_INFO(this->get_logger(), "  Monitoring Servo ID: %d", servo_id);
        
        // Initialize IoContext
        auto io_ctx = std::make_shared<IoContext>();
        
        // Initialize LX16a
        auto lx16a = std::make_shared<LX16a>(io_ctx);
        if (!lx16a->open(port)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        
        // Run diagnostic
        run_diagnostic(lx16a, servo_id, measurement_duration);
    }

private:
    void run_diagnostic(std::shared_ptr<LX16a> lx16a, int servo_id, double measurement_duration)
    {
        std::vector<double> cycle_times_ms;
        std::vector<int> positions;
        
        RCLCPP_INFO(this->get_logger(), "\n========== DIAGNOSTIC STARTED ==========");
        RCLCPP_INFO(this->get_logger(), "Querying servo %d position repeatedly for %.1f seconds...\n", servo_id, measurement_duration);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        auto end_time = start_time + std::chrono::duration<double>(measurement_duration);
        
        int16_t pos = 0;
        auto last_pos_time = std::chrono::high_resolution_clock::now();
        int query_count = 0;
        
        while (std::chrono::high_resolution_clock::now() < end_time) {
            if (lx16a->get_pos(servo_id, pos)) {
                auto current_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> elapsed_ms = current_time - last_pos_time;
                
                if (query_count > 0) {  // Skip first measurement
                    cycle_times_ms.push_back(elapsed_ms.count());
                    positions.push_back(pos);
                }
                
                last_pos_time = current_time;
                query_count++;
                
                // Print progress every 100 queries
                if (query_count % 100 == 0) {
                    RCLCPP_INFO(this->get_logger(), "Queries: %d, Last Cycle: %.2f ms, Position: %d", 
                        query_count, elapsed_ms.count(), pos);
                }
            }
        }
        
        auto total_duration = std::chrono::high_resolution_clock::now() - start_time;
        
        // Calculate statistics
        if (cycle_times_ms.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No data collected! Check servo connection.");
            return;
        }
        
        double min_cycle = *std::min_element(cycle_times_ms.begin(), cycle_times_ms.end());
        double max_cycle = *std::max_element(cycle_times_ms.begin(), cycle_times_ms.end());
        double avg_cycle = std::accumulate(cycle_times_ms.begin(), cycle_times_ms.end(), 0.0) / cycle_times_ms.size();
        double hz = 1000.0 / avg_cycle;
        
        // Calculate standard deviation
        double variance = 0.0;
        for (double time : cycle_times_ms) {
            variance += (time - avg_cycle) * (time - avg_cycle);
        }
        variance /= cycle_times_ms.size();
        double std_dev = std::sqrt(variance);
        
        // Print results
        RCLCPP_INFO(this->get_logger(), "\n========== DIAGNOSTIC RESULTS ==========");
        RCLCPP_INFO(this->get_logger(), "Total Measurement Time: %.3f seconds", total_duration.count() / 1e9);
        RCLCPP_INFO(this->get_logger(), "Total Queries: %d", query_count);
        RCLCPP_INFO(this->get_logger(), "Valid Measurements: %zu\n", cycle_times_ms.size());
        
        RCLCPP_INFO(this->get_logger(), "Cycle Time Statistics:");
        RCLCPP_INFO(this->get_logger(), "  Average: %.3f ms", avg_cycle);
        RCLCPP_INFO(this->get_logger(), "  Min:     %.3f ms", min_cycle);
        RCLCPP_INFO(this->get_logger(), "  Max:     %.3f ms", max_cycle);
        RCLCPP_INFO(this->get_logger(), "  Std Dev: %.3f ms\n", std_dev);
        
        RCLCPP_INFO(this->get_logger(), "SERVO CONTROLLER FREQUENCY:");
        RCLCPP_INFO(this->get_logger(), "  *** %.2f Hz ***\n", hz);
        
        RCLCPP_INFO(this->get_logger(), "Position Range: %d to %d", 
            *std::min_element(positions.begin(), positions.end()),
            *std::max_element(positions.begin(), positions.end()));
        RCLCPP_INFO(this->get_logger(), "========================================\n");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoControllerDiagnosticsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
