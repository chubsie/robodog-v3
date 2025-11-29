/////////////////////////////////////////////////////////////////////////////////
// MIT License
//
// Copyright (c) 2020 RoboLab19
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////
#include "RemoteController.h"
#include <iostream>
#include <filesystem>
#include <algorithm>

RemoteController::RemoteController() : Node("remote_controller"),
    m_button_b(0), m_button_a(1), m_button_y(2), m_button_x(3),
    m_button_l(4), m_button_r(5), m_button_l2(6), m_button_r2(7),
    m_button_minus(8), m_button_plus(9), m_button_l_stick(10), m_button_r_stick(11),
    m_button_home(12), m_button_star(13),
    m_left_deadzone(0.15), m_right_deadzone(0.15)
{
    // Initialize servo controller
    init_servo_controller();
    
    // Try to load configuration file
    std::string config_file = "controller_config.yaml";
    
    // Check multiple possible locations for config file
    std::vector<std::string> config_paths = {
        config_file,
        std::string(getenv("HOME")) + "/.ros/controller_config.yaml",
        "/opt/ros/humble/share/robodog/config/controller_config.yaml",
        std::filesystem::current_path().string() + "/config/" + config_file
    };
    
    bool config_loaded = false;
    for (const auto& path : config_paths) {
        if (std::filesystem::exists(path)) {
            if (load_controller_config(path)) {
                config_loaded = true;
                RCLCPP_INFO(this->get_logger(), "Loaded controller config from: %s", path.c_str());
                break;
            }
        }
    }
    
    if (!config_loaded) {
        RCLCPP_WARN(this->get_logger(), "Could not load controller config file, using defaults");
    }

    m_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&RemoteController::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RemoteController initialized");
    RCLCPP_INFO(this->get_logger(), "Servo control enabled: %s", m_enable_servo_control ? "YES" : "NO");
}

bool RemoteController::load_controller_config(const std::string& config_file)
{
    try {
        m_config = YAML::LoadFile(config_file);
        
        // Load button mappings
        if (m_config["buttons"]) {
            YAML::Node buttons = m_config["buttons"];
            for (YAML::const_iterator it = buttons.begin(); it != buttons.end(); ++it) {
                std::string key = it->first.as<std::string>();
                int value = it->second.as<int>();
                m_button_mapping[key] = value;
                RCLCPP_DEBUG(this->get_logger(), "Button mapping: %s -> %d", key.c_str(), value);
            }
        }
        
        // Load axis mappings
        if (m_config["sticks"]) {
            YAML::Node sticks = m_config["sticks"];
            if (sticks["left_deadzone"]) {
                m_left_deadzone = sticks["left_deadzone"].as<float>();
            }
            if (sticks["right_deadzone"]) {
                m_right_deadzone = sticks["right_deadzone"].as<float>();
            }
        }
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading config: %s", e.what());
        return false;
    }
}

void RemoteController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Safety check - ensure we have enough buttons and axes
    if (msg->buttons.size() < 14) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "Received joy message with only %zu buttons", msg->buttons.size());
        return;
    }
    
    if (msg->axes.size() < 6) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "Received joy message with only %zu axes", msg->axes.size());
        return;
    }
    
    // Update button states
    m_button_b.check(msg->buttons[0]);
    m_button_a.check(msg->buttons[1]);
    m_button_y.check(msg->buttons[2]);
    m_button_x.check(msg->buttons[3]);
    m_button_l.check(msg->buttons[4]);
    m_button_r.check(msg->buttons[5]);
    m_button_l2.check(msg->buttons[6]);
    m_button_r2.check(msg->buttons[7]);
    m_button_minus.check(msg->buttons[8]);
    m_button_plus.check(msg->buttons[9]);
    m_button_l_stick.check(msg->buttons[10]);
    m_button_r_stick.check(msg->buttons[11]);
    m_button_home.check(msg->buttons[12]);
    m_button_star.check(msg->buttons[13]);

    // Process analog stick inputs with deadzone
    float left_x = msg->axes[0];   // Left stick horizontal
    float left_y = msg->axes[1];   // Left stick vertical
    float right_x = msg->axes[2];  // Right stick horizontal
    float right_y = msg->axes[3];  // Right stick vertical
    
    // Apply deadzones
    if (std::abs(left_x) < m_left_deadzone) left_x = 0.0f;
    if (std::abs(left_y) < m_left_deadzone) left_y = 0.0f;
    if (std::abs(right_x) < m_right_deadzone) right_x = 0.0f;
    if (std::abs(right_y) < m_right_deadzone) right_y = 0.0f;

    // Button event handling for servo control
    if (m_button_minus.clicked()) {
        m_enable_servo_control = !m_enable_servo_control;
        RCLCPP_INFO(this->get_logger(), "Servo control %s", 
                   m_enable_servo_control ? "ENABLED" : "DISABLED");
    }
    
    if (m_button_plus.clicked()) {
        m_servo_speed_multiplier = std::max(0.5f, m_servo_speed_multiplier - 0.1f);
        RCLCPP_INFO(this->get_logger(), "Servo speed: %.1f%%", m_servo_speed_multiplier * 100.0f);
    }
    
    if (m_button_l_stick.clicked()) {
        m_servo_speed_multiplier = 1.0f;
        RCLCPP_INFO(this->get_logger(), "Servo speed reset to 100%%");
    }
    
    if (m_button_r.clicked()) {
        m_servo_speed_multiplier = std::min(2.0f, m_servo_speed_multiplier + 0.1f);
        RCLCPP_INFO(this->get_logger(), "Servo speed: %.1f%%", m_servo_speed_multiplier * 100.0f);
    }

    // Servo control with analog sticks (if enabled)
    if (m_enable_servo_control) {
        // Left stick: servo 1 (vertical) and servo 2 (horizontal)
        if (left_y != 0.0f) {
            move_servo(1, left_y);
        }
        if (left_x != 0.0f) {
            move_servo(2, left_x);
        }
        
        // Right stick: servo 3 (vertical) and servo 4 (horizontal)
        if (right_y != 0.0f) {
            move_servo(3, right_y);
        }
        if (right_x != 0.0f) {
            move_servo(4, right_x);
        }
        
        // D-Pad: servo 5 and 6
        float dpad_x = msg->axes[4];
        float dpad_y = msg->axes[5];
        if (dpad_y != 0.0f) {
            move_servo(5, dpad_y);
        }
        if (dpad_x != 0.0f) {
            move_servo(6, dpad_x);
        }
    }

    // Log joystick values for debugging (throttled)
    if ((left_x != 0.0f || left_y != 0.0f) && m_enable_servo_control) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "L_X: %.2f, L_Y: %.2f", left_x, left_y);
    }
}

void RemoteController::init_servo_controller()
{
    try {
        // Create IoContext for serial communication
        auto io_ctx = std::make_shared<IoContext>();
        
        // Create LX16a servo controller
        m_servo_controller = std::make_shared<LX16a>(io_ctx);
        
        // Try to open serial port
        if (!m_servo_controller->open("/dev/ttyUSB0")) {
            RCLCPP_WARN(this->get_logger(), "Failed to open /dev/ttyUSB0, trying /dev/ttyACM0");
            if (!m_servo_controller->open("/dev/ttyACM0")) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open servo serial port on either /dev/ttyUSB0 or /dev/ttyACM0");
                m_enable_servo_control = false;
                return;
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Servo controller initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception initializing servo controller: %s", e.what());
        m_enable_servo_control = false;
    }
}

void RemoteController::move_servo(int servo_id, float analog_value)
{
    if (!m_servo_controller || !m_enable_servo_control) {
        return;
    }
    
    try {
        // Map analog value (-1.0 to 1.0) to servo position (0 to 1000)
        // Center at 500 (0.5 * 1000)
        int position_int = 500 + static_cast<int>(analog_value * 500.0f);
        uint16_t position = std::max(0, std::min(1000, position_int));
        
        // Apply speed multiplier to move time
        uint16_t move_time = static_cast<uint16_t>(m_servo_move_time / m_servo_speed_multiplier);
        move_time = std::max(static_cast<uint16_t>(10), move_time);  // Minimum 10ms move time
        
        // Send position command using LX16a move method
        if (!m_servo_controller->move(servo_id, position, move_time)) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Failed to move servo %d to position %d", servo_id, position);
            return;
        }
        
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "Servo %d -> Position: %d (analog: %.2f, move_time: %dms)",
                             servo_id, position, analog_value, move_time);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Error moving servo %d: %s", servo_id, e.what());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RemoteController>());
    rclcpp::shutdown();
    return 0;
}

