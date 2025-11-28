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
#include <yaml-cpp/yaml.h>
#include <map>
#include <string>


class RemoteController : public rclcpp::Node
{
public:
    RemoteController();

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    bool load_controller_config(const std::string& config_file);
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_sub;

    // Dynamic button storage based on config
    std::map<std::string, std::shared_ptr<Button>> m_buttons;
    
    // Configuration storage
    YAML::Node m_config;
    std::map<std::string, int> m_button_mapping;
    std::map<std::string, int> m_axis_mapping;

    // Button state tracking
    Button m_button_a;
    Button m_button_b;
    Button m_button_x;
    Button m_button_y;
    Button m_button_l1;
    Button m_button_r1;
    Button m_button_back;
    Button m_button_start;
    Button m_button_l3;
    Button m_button_r3;

    // Configuration values
    float m_left_deadzone;
    float m_right_deadzone;
};


