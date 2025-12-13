/////////////////////////////////////////////////////////////////////////////////
// RoboDog V3 - Remote Controller Implementation
// 
// This implementation uses GCode primitives for walking gaits.
// It loads button mappings from config/controller_config.yaml.
// 
// WALKING GAIT: Uses $WALK_3 primitive for continuous trot gait
// The robot uses 20 motors, 12 DOF (4 legs x 3 joints)
// Hip and Knee joints use dual motors (JointDouble) facing each other
//
// COORDINATE SYSTEMS:
// - GCode (used by walking): X=forward, Y=height(+up), Z=sideways
// - Leg IK (direct leg API): X=forward, Y=outward, Z=height(-down)
/////////////////////////////////////////////////////////////////////////////////

#include "RemoteController.h"
#include "Factory.h"
#include "Utils.h"
#include "GCodeController.h"
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <sstream>

// Configuration constants
static const int BASE_MOVE_TIME = 1000;     // Base movement time in milliseconds
static const float MIN_SPEED = 0.1f;
static const float MAX_SPEED = 2.0f;
static const float SPEED_INCREMENT = 0.1f;

// Walking gait parameters (in mm for GCode system)
// Larger step sizes = more actual locomotion distance per step cycle
static const int WALK_STEP_SIZE = 60;       // Forward/back step size in mm (was 30)
static const int WALK_SIDE_STEP = 40;       // Side step size in mm (was 20)
static const int WALK_ROTATE_STEP = 15;     // Rotation step in degrees (was 10)

RemoteController::RemoteController() : Node("robodog_remote_controller"),
    m_button_speed_down(0), m_button_speed_up(1), m_button_sit(2), m_button_stand(3),
    m_button_servo_toggle(4), m_button_emergency(8), m_button_resume(9), m_button_speed_reset(10)
{
    // Initialize rate limiting timestamp
    m_last_walk_command_time = this->get_clock()->now();
    
    // Initialize default mappings (Nintendo Pro Controller / 8bitdo Ultimate Switch Mode)
    m_button_mapping = {
        {"speed_down", 0},
        {"speed_up", 1},
        {"sit", 2},
        {"stand", 3},
        {"servo_toggle", 4},
        {"emergency_stop", 8},
        {"resume", 9},
        {"speed_reset", 10}
    };
    
    m_axis_mapping = {
        {"left_stick_x", 0},
        {"left_stick_y", 1},
        {"right_stick_x", 2},
        {"right_stick_y", 3}
    };

    // Try to load configuration file
    std::string config_file = "controller_config.yaml";
    std::vector<std::string> config_paths = {
        config_file,
        std::string(getenv("HOME")) + "/.ros/controller_config.yaml",
        "/opt/ros/humble/share/robodog/config/controller_config.yaml",
        std::filesystem::current_path().string() + "/config/" + config_file,
        "/home/chubsie/ros2_ws/src/RoboDog_v3_ros2/config/" + config_file // Explicit workspace path
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

    // Create robot
    if (!Factory::CreateRobotV3(m_robot)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create RobotV3");
        throw std::runtime_error("Robot creation failed");
    }
    
    // Get reference to legs
    m_legs = m_robot->GetGcodeCtrl()->GetLegs();
    
    if (m_legs.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No legs found in robot");
        throw std::runtime_error("No legs initialized");
    }
    
    RCLCPP_INFO(this->get_logger(), "Robot initialized with %zu legs", m_legs.size());

    // Initialize robot to STAND position using GCode
    // This uses $POSE 1 which is the "Middle" position (Y=200mm height)
    RCLCPP_INFO(this->get_logger(), "Initializing robot to STAND position...");
    auto gcode = m_robot->GetGcodeCtrl();
    gcode->run("S 50\n");  // Set speed to 50%
    gcode->run("$POSE 1\n");  // Set pose 1 = Middle/Stand (M 0 200 0)
    m_robot_initialized = true;
    RCLCPP_INFO(this->get_logger(), "Robot initialized to STAND position");

    // Subscribe to Joy messages
    m_joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&RemoteController::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RemoteController ready - waiting for Joy input...");
    RCLCPP_INFO(this->get_logger(), "Controls: Left stick = Walk forward/back/strafe, Right stick X = Rotate");
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
            if (sticks["left_deadzone"]) m_left_deadzone = sticks["left_deadzone"].as<float>();
            if (sticks["right_deadzone"]) m_right_deadzone = sticks["right_deadzone"].as<float>();
            
            if (sticks["left_stick_x"]) m_axis_mapping["left_stick_x"] = sticks["left_stick_x"].as<int>();
            if (sticks["left_stick_y"]) m_axis_mapping["left_stick_y"] = sticks["left_stick_y"].as<int>();
            if (sticks["right_stick_x"]) m_axis_mapping["right_stick_x"] = sticks["right_stick_x"].as<int>();
            if (sticks["right_stick_y"]) m_axis_mapping["right_stick_y"] = sticks["right_stick_y"].as<int>();
        }
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error loading config: %s", e.what());
        return false;
    }
}

void RemoteController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    // Safety check
    if (msg->buttons.size() < 10 || msg->axes.size() < 4) {
        return;
    }
    
    // Helper to safely get button state
    auto get_btn = [&](const std::string& name) -> bool {
        if (m_button_mapping.find(name) != m_button_mapping.end()) {
            int idx = m_button_mapping[name];
            if (idx >= 0 && idx < (int)msg->buttons.size()) {
                return msg->buttons[idx] != 0;
            }
        }
        return false;
    };

    // Update button states
    m_button_speed_down.check(get_btn("speed_down"));
    m_button_speed_up.check(get_btn("speed_up"));
    m_button_sit.check(get_btn("sit"));
    m_button_stand.check(get_btn("stand"));
    m_button_servo_toggle.check(get_btn("servo_toggle"));
    m_button_emergency.check(get_btn("emergency_stop"));
    m_button_resume.check(get_btn("resume"));
    m_button_speed_reset.check(get_btn("speed_reset"));

    // Handle Actions
    
    // Speed Control
    if (m_button_speed_down.clicked()) {
        m_speed_multiplier = std::max(MIN_SPEED, m_speed_multiplier - SPEED_INCREMENT);
        RCLCPP_INFO(this->get_logger(), "Speed: %.1fx", m_speed_multiplier);
    }
    if (m_button_speed_up.clicked()) {
        m_speed_multiplier = std::min(MAX_SPEED, m_speed_multiplier + SPEED_INCREMENT);
        RCLCPP_INFO(this->get_logger(), "Speed: %.1fx", m_speed_multiplier);
    }
    if (m_button_speed_reset.clicked()) {
        m_speed_multiplier = 1.0f;
        RCLCPP_INFO(this->get_logger(), "Speed reset: 1.0x");
    }

    // Posture Control
    if (m_button_sit.clicked()) {
        move_to_sit();
    }
    if (m_button_stand.clicked()) {
        move_to_stand();
    }

    // Safety Control
    if (m_button_servo_toggle.clicked()) {
        m_servos_enabled = !m_servos_enabled;
        RCLCPP_INFO(this->get_logger(), "Servos: %s", m_servos_enabled ? "ENABLED" : "DISABLED");
        if (m_servos_enabled) move_to_stand();
    }
    if (m_button_emergency.clicked()) {
        m_movement_enabled = false;
        RCLCPP_INFO(this->get_logger(), "Movement DISABLED (emergency stop)");
    }
    if (m_button_resume.clicked()) {
        m_movement_enabled = true;
        RCLCPP_INFO(this->get_logger(), "Movement ENABLED");
    }

    // Analog Stick Processing
    if (!m_movement_enabled || !m_servos_enabled) return;

    // Get axes based on mapping
    int idx_lx = m_axis_mapping["left_stick_x"];
    int idx_ly = m_axis_mapping["left_stick_y"];
    int idx_rx = m_axis_mapping["right_stick_x"];
    int idx_ry = m_axis_mapping["right_stick_y"];

    float left_x = (idx_lx < (int)msg->axes.size()) ? apply_deadzone(msg->axes[idx_lx]) : 0.0f;
    float left_y = (idx_ly < (int)msg->axes.size()) ? apply_deadzone(msg->axes[idx_ly]) : 0.0f;
    float right_x = (idx_rx < (int)msg->axes.size()) ? apply_deadzone(msg->axes[idx_rx]) : 0.0f;
    float right_y = (idx_ry < (int)msg->axes.size()) ? apply_deadzone(msg->axes[idx_ry]) : 0.0f;

    if (left_x != 0.0f || left_y != 0.0f || right_x != 0.0f || right_y != 0.0f) {
        process_movement(left_x, left_y, right_x, right_y);
    }
}

float RemoteController::apply_deadzone(float value)
{
    // Use the larger of the two deadzones for simplicity, or specific ones if needed
    float deadzone = std::max(m_left_deadzone, m_right_deadzone);
    if (std::abs(value) < deadzone) return 0.0f;
    return value;
}

void RemoteController::process_movement(float left_x, float left_y, float right_x, float right_y)
{
    // Check if robot is still executing previous walk command
    // Only allow new command when trajectory queue is nearly empty
    auto gcode = m_robot->GetGcodeCtrl();
    auto finish_time = gcode->GetMoveFinishTime();
    // Use system clock (same as GCodeController uses internally)
    auto now = rclcpp::Clock().now();
    
    // Allow new command if previous one will finish within 100ms
    // This creates overlapping commands for smooth continuous walking
    double remaining = 0.0;
    if (finish_time > now) {
        remaining = (finish_time - now).seconds();
    }
    if (remaining > 0.1) {
        // Still executing previous command, skip this one
        return;
    }
    
    // Use GCode $WALK_4 for walking gait with swing leg trajectories
    // $WALK_4 params: forward_step (mm), side_step (mm)
    // Legs swing through arc trajectories for locomotion
    
    // Map Joy input to walking parameters
    // Joy conventions vary by controller. For 8BitDo/Nintendo:
    // - Push stick forward = positive Y (some controllers = negative Y)
    // - Push stick left = negative X
    // Adjust signs here based on your controller behavior:
    int forward_step = static_cast<int>(left_y * WALK_STEP_SIZE * m_speed_multiplier);  // Inverted: + = forward
    int side_step = static_cast<int>(-left_x * WALK_SIDE_STEP * m_speed_multiplier);    // Inverted: - = left
    int rotate_step = static_cast<int>(-right_x * WALK_ROTATE_STEP * m_speed_multiplier);  // - = turn left
    
    // Minimum step thresholds (robot needs some minimum step to actually move)
    const int MIN_FORWARD_STEP = 20;  // $WALK_4 enforces min 20mm internally
    const int MIN_SIDE_STEP = 10;     // $WALK_4 enforces min 10mm internally
    const int MIN_ROTATE_STEP = 3;
    
    // Apply minimum step thresholds - if below minimum, clamp to minimum in the same direction
    if (forward_step != 0 && std::abs(forward_step) < MIN_FORWARD_STEP) {
        forward_step = (forward_step > 0) ? MIN_FORWARD_STEP : -MIN_FORWARD_STEP;
    }
    if (side_step != 0 && std::abs(side_step) < MIN_SIDE_STEP) {
        side_step = (side_step > 0) ? MIN_SIDE_STEP : -MIN_SIDE_STEP;
    }
    if (rotate_step != 0 && std::abs(rotate_step) < MIN_ROTATE_STEP) {
        rotate_step = (rotate_step > 0) ? MIN_ROTATE_STEP : -MIN_ROTATE_STEP;
    }
    
    // If no significant input, skip
    if (forward_step == 0 && side_step == 0 && rotate_step == 0) {
        return;
    }
    
    // Log what we're doing
    RCLCPP_INFO(this->get_logger(), "Walking: forward=%d side=%d rotate=%d remaining=%.3fs", 
                forward_step, side_step, rotate_step, remaining);
    
    // Execute the walking gait using GCode
    // (gcode already obtained above for finish time check)
    
    // Build GCode command: $WALK_4 forward_step side_step
    // $WALK_4 uses swing leg trajectories with proper foot arcs
    std::string walk_cmd = "$WALK_4 " + std::to_string(forward_step) + " " 
                                      + std::to_string(side_step) + "\n";
    
    // Set rotation via BASE_ROT for turning
    if (rotate_step != 0) {
        std::string rot_cmd = "BASE_ROT 0 0 " + std::to_string(rotate_step) + "\n";
        gcode->run(rot_cmd);
    }
    
    if (!gcode->run(walk_cmd)) {
        RCLCPP_WARN(this->get_logger(), "Walk command failed: %s", walk_cmd.c_str());
    }
    
    // Reset rotation after step
    if (rotate_step != 0) {
        gcode->run("BASE_ROT 0 0 0\n");
    }
}

void RemoteController::move_to_stand()
{
    RCLCPP_INFO(this->get_logger(), "Moving to STAND position...");
    
    // Use GCode $POSE 1 for standing (M 0 200 0 = height 200mm)
    auto gcode = m_robot->GetGcodeCtrl();
    gcode->run("S 50\n");     // Set speed to 50%
    gcode->run("$POSE 1\n");  // Pose 1 = Middle/Stand
    
    m_robot_initialized = true;
    RCLCPP_INFO(this->get_logger(), "Robot in STAND position");
}

void RemoteController::move_to_sit()
{
    RCLCPP_INFO(this->get_logger(), "Moving to SIT position...");
    
    // Use GCode $POSE 0 for sitting (M 0 80 0 = height 80mm)
    auto gcode = m_robot->GetGcodeCtrl();
    gcode->run("S 50\n");     // Set speed to 50%
    gcode->run("$POSE 0\n");  // Pose 0 = Down/Sit
    
    RCLCPP_INFO(this->get_logger(), "Robot in SIT position");
}
