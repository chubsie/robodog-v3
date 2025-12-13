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
// OUT of OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "Button.h"
#include "CommandMenu.h"
#include "IK.h"
#include "Robot.h"
#include "GCodeController.h"
#include "Servos/LX16a.h"
#include "io_context/io_context.hpp"
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>
#include <memory>
#include <deque>

using drivers::common::IoContext;


class RemoteController : public rclcpp::Node
{
public:
    RemoteController();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    bool load_controller_config(const std::string& config_file);
    
    // Movement control methods
    void process_movement(float left_x, float left_y, float right_x, float right_y);
    void move_to_stand();
    void move_to_sit();
    float apply_deadzone(float value);
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_sub;

    // Robot components
    std::shared_ptr<Robot> m_robot;
    GCodeController::RobotLegs m_legs;
    
    // State tracking
    float m_speed_multiplier = 1.0f;
    bool m_movement_enabled = true;
    bool m_servos_enabled = true;
    bool m_robot_initialized = false;  // Track if robot has been initialized to stand
    
    // Current foot position tracking (leg-local coordinates)
    double m_foot_x = 0.0;
    double m_foot_y = 0.10;
    double m_foot_z = -0.20;

    // Dynamic button storage based on config
    std::map<std::string, std::shared_ptr<Button>> m_buttons;
    
    // Configuration storage
    YAML::Node m_config;
    std::map<std::string, int> m_button_mapping;
    std::map<std::string, int> m_axis_mapping;
    
    // Deadzones
    float m_left_deadzone = 0.15f;
    float m_right_deadzone = 0.15f;

    // Rate limiting for walking commands
    rclcpp::Time m_last_walk_command_time;
    bool m_walking_in_progress = false;

    // Button state tracking
    Button m_button_speed_down;     // B
    Button m_button_speed_up;       // A
    Button m_button_sit;            // Y
    Button m_button_stand;          // X
    Button m_button_servo_toggle;   // L
    Button m_button_emergency;      // Minus
    Button m_button_resume;         // Plus
    Button m_button_speed_reset;    // L-Stick Click
};


