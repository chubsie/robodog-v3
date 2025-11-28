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

RemoteController::RemoteController() : Node("remote_controller"),
    m_button_a(0), m_button_b(1), m_button_x(2), m_button_y(3),
    m_button_l1(4), m_button_r1(5), m_button_back(6), m_button_start(7),
    m_button_l3(9), m_button_r3(10),
    m_left_deadzone(0.15), m_right_deadzone(0.15)
{
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
    // Safety check - ensure we have enough buttons
    if (msg->buttons.size() < 11) {
        RCLCPP_WARN(this->get_logger(), "Received joy message with only %zu buttons", msg->buttons.size());
        return;
    }
    
    // Check buttons with mapped indices
    m_button_a.check(msg->buttons[0]);
    m_button_b.check(msg->buttons[1]);
    m_button_x.check(msg->buttons[2]);
    m_button_y.check(msg->buttons[3]);
    m_button_l1.check(msg->buttons[4]);
    m_button_r1.check(msg->buttons[5]);
    m_button_back.check(msg->buttons[6]);
    m_button_start.check(msg->buttons[7]);
    m_button_l3.check(msg->buttons[9]);
    m_button_r3.check(msg->buttons[10]);

    // Safety check for axes
    if (msg->axes.size() < 6) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "Received joy message with only %zu axes", msg->axes.size());
        return;
    }

    // Process analog stick inputs with deadzone
    float left_x = msg->axes[0];   // Left stick horizontal
    float left_y = msg->axes[1];   // Left stick vertical (inverted by joy_node)
    float right_x = msg->axes[3];  // Right stick horizontal
    
    // Apply deadzones
    if (std::abs(left_x) < m_left_deadzone) left_x = 0.0f;
    if (std::abs(left_y) < m_left_deadzone) left_y = 0.0f;
    if (std::abs(right_x) < m_right_deadzone) right_x = 0.0f;

    // Log joystick values for debugging (throttled)
    if (left_x != 0.0f || left_y != 0.0f || right_x != 0.0f) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Joystick - L_X: %.2f, L_Y: %.2f, R_X: %.2f",
                             left_x, left_y, right_x);
    }

    // Button event handling
    if (m_button_a.clicked()) {
        RCLCPP_INFO(this->get_logger(), "Button A pressed - Stand mode");
    }
    if (m_button_b.clicked()) {
        RCLCPP_INFO(this->get_logger(), "Button B pressed - Walk mode");
    }
    if (m_button_x.clicked()) {
        RCLCPP_INFO(this->get_logger(), "Button X pressed - Trot mode");
    }
    if (m_button_y.clicked()) {
        RCLCPP_INFO(this->get_logger(), "Button Y pressed - Gallop mode");
    }
    
    if (m_button_l1.clicked()) {
        RCLCPP_INFO(this->get_logger(), "LB pressed - Speed down");
    }
    if (m_button_r1.clicked()) {
        RCLCPP_INFO(this->get_logger(), "RB pressed - Speed up");
    }
    
    if (m_button_back.clicked()) {
        RCLCPP_INFO(this->get_logger(), "Back button pressed - Reset");
    }
    if (m_button_start.clicked()) {
        RCLCPP_INFO(this->get_logger(), "Start button pressed");
    }
    
    if (m_button_l3.clicked()) {
        RCLCPP_INFO(this->get_logger(), "Left stick click - Debug toggle");
    }
    if (m_button_r3.clicked()) {
        RCLCPP_INFO(this->get_logger(), "Right stick click - Diagnostics");
    }

    // TODO: Integrate with actual robot movement commands
    // Send movement commands to the robot based on joystick input
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RemoteController>());
    rclcpp::shutdown();
    return 0;
}

