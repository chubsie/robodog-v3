/////////////////////////////////////////////////////////////////////////////////
// RoboDog V3 - Proper Joy Controller Implementation
// 
// This implementation uses the Leg API with Inverse Kinematics instead of
// direct servo control. Movement is synchronized across all four legs with
// proper physics.
//
// Control Flow:
//   Joy Input (analog sticks, buttons)
//     ↓
//   Map to desired robot velocity/orientation
//     ↓
//   For each leg:
//     Calculate desired foot position based on body motion
//     Use Inverse Kinematics to convert to joint angles
//     Send angles to Leg::move() which handles joint synchronization
//     JointDouble pairs handle opposite-facing servo inversions
//     ↓
//   Synchronized multi-leg movement with proper physics
/////////////////////////////////////////////////////////////////////////////////

#include "RemoteController.h"
#include "Factory.h"
#include "Utils.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <chrono>
#include <cmath>

// Configuration constants
static const float DEADZONE = 0.15f;  // 15% deadzone to filter stick noise
static const float MIN_SPEED = 0.1f;   // Minimum speed multiplier (10%)
static const float MAX_SPEED = 2.0f;   // Maximum speed multiplier (200%)
static const float SPEED_INCREMENT = 0.1f;  // Speed change per button press
static const int BASE_MOVE_TIME = 1000;     // Base movement time in milliseconds

// Robot geometry - set during initialization
static const double LEG_X_OFFSET = 0.209;  // Front legs distance from center
static const double LEG_X_BACK = -0.209;   // Back legs distance from center
static const double LEG_Y_LEFT = 0.115;    // Left legs lateral distance
static const double LEG_Y_RIGHT = -0.115;  // Right legs lateral distance

class RemoteControllerImpl : public rclcpp::Node
{
public:
    RemoteControllerImpl() : rclcpp::Node("robodog_remote_controller")
    {
        m_speed_multiplier = 1.0f;
        m_movement_enabled = true;
        m_servos_enabled = true;
        m_last_joy_state = {};
        
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
        
        // Subscribe to Joy messages
        m_joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&RemoteControllerImpl::joy_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Remote controller ready - waiting for Joy input...");
    }

private:
    std::shared_ptr<Robot> m_robot;
    GCodeController::RobotLegs m_legs;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_subscription;
    
    float m_speed_multiplier;
    bool m_movement_enabled;
    bool m_servos_enabled;
    sensor_msgs::msg::Joy m_last_joy_state;
    
    // Current foot position tracking (leg-local coordinates)
    double m_foot_x = 0.0;
    double m_foot_y = 0.10;
    double m_foot_z = -0.20;
    
    // Button state tracking for edge detection
    struct ButtonState {
        bool a_pressed = false;
        bool b_pressed = false;
        bool x_pressed = false;
        bool y_pressed = false;
        bool l_pressed = false;
        bool minus_pressed = false;
        bool plus_pressed = false;
        bool l_stick_pressed = false;
    } m_button_state;
    
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!msg || msg->buttons.size() < 14 || msg->axes.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Invalid Joy message format");
            return;
        }
        
        // Extract analog stick values with deadzone filtering
        float left_x = apply_deadzone(msg->axes[0]);
        float left_y = apply_deadzone(msg->axes[1]);
        float right_x = apply_deadzone(msg->axes[2]);
        float right_y = apply_deadzone(msg->axes[3]);
        
        // Handle button presses (edge detection)
        handle_button_presses(msg);
        
        // If movement is disabled, don't process stick input
        if (!m_movement_enabled) {
            return;
        }
        
        if (!m_servos_enabled) {
            return;
        }
        
        // Only process if there's actual input
        if (left_x != 0.0f || left_y != 0.0f || right_x != 0.0f || right_y != 0.0f) {
            process_movement(left_x, left_y, right_x, right_y);
        }
        
        m_last_joy_state = *msg;
    }
    
    float apply_deadzone(float value)
    {
        if (std::abs(value) < DEADZONE) {
            return 0.0f;
        }
        return value;
    }
    
    void handle_button_presses(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // B Button (Index 0) - Speed Down
        if (msg->buttons[0] && !m_button_state.b_pressed) {
            m_speed_multiplier = std::max(MIN_SPEED, m_speed_multiplier - SPEED_INCREMENT);
            RCLCPP_INFO(this->get_logger(), "Speed: %.1fx", m_speed_multiplier);
            m_button_state.b_pressed = true;
        } else if (!msg->buttons[0]) {
            m_button_state.b_pressed = false;
        }
        
        // A Button (Index 1) - Speed Up
        if (msg->buttons[1] && !m_button_state.a_pressed) {
            m_speed_multiplier = std::min(MAX_SPEED, m_speed_multiplier + SPEED_INCREMENT);
            RCLCPP_INFO(this->get_logger(), "Speed: %.1fx", m_speed_multiplier);
            m_button_state.a_pressed = true;
        } else if (!msg->buttons[1]) {
            m_button_state.a_pressed = false;
        }
        
        // Y Button (Index 2) - Sit Position
        if (msg->buttons[2] && !m_button_state.y_pressed) {
            RCLCPP_INFO(this->get_logger(), "Moving to SIT position...");
            move_to_sit();
            m_button_state.y_pressed = true;
        } else if (!msg->buttons[2]) {
            m_button_state.y_pressed = false;
        }
        
        // X Button (Index 3) - Stand Position
        if (msg->buttons[3] && !m_button_state.x_pressed) {
            RCLCPP_INFO(this->get_logger(), "Moving to STAND position...");
            move_to_stand();
            m_button_state.x_pressed = true;
        } else if (!msg->buttons[3]) {
            m_button_state.x_pressed = false;
        }
        
        // L Button (Index 4) - Toggle Servo Enable/Disable
        if (msg->buttons[4] && !m_button_state.l_pressed) {
            m_servos_enabled = !m_servos_enabled;
            RCLCPP_INFO(this->get_logger(), "Servos: %s", m_servos_enabled ? "ENABLED" : "DISABLED");
            
            if (m_servos_enabled) {
                // Re-initialize all legs to current positions
                move_to_stand();
            }
            
            m_button_state.l_pressed = true;
        } else if (!msg->buttons[4]) {
            m_button_state.l_pressed = false;
        }
        
        // Minus Button (Index 8) - Disable Movement
        if (msg->buttons[8] && !m_button_state.minus_pressed) {
            m_movement_enabled = false;
            RCLCPP_INFO(this->get_logger(), "Movement DISABLED (emergency stop)");
            m_button_state.minus_pressed = true;
        } else if (!msg->buttons[8]) {
            m_button_state.minus_pressed = false;
        }
        
        // Plus Button (Index 9) - Enable Movement
        if (msg->buttons[9] && !m_button_state.plus_pressed) {
            m_movement_enabled = true;
            RCLCPP_INFO(this->get_logger(), "Movement ENABLED");
            m_button_state.plus_pressed = true;
        } else if (!msg->buttons[9]) {
            m_button_state.plus_pressed = false;
        }
        
        // L-Stick Click (Index 10) - Reset Speed
        if (msg->buttons[10] && !m_button_state.l_stick_pressed) {
            m_speed_multiplier = 1.0f;
            RCLCPP_INFO(this->get_logger(), "Speed reset: 1.0x");
            m_button_state.l_stick_pressed = true;
        } else if (!msg->buttons[10]) {
            m_button_state.l_stick_pressed = false;
        }
    }
    
    void process_movement(float left_x, float left_y, float right_x, float right_y)
    {
        // Map Joy input to robot body motion
        // Left Stick Y (negative = forward, positive = backward)
        // Left Stick X (negative = left strafe, positive = right strafe)
        // Right Stick X (negative = turn left, positive = turn right)
        // Right Stick Y (negative = raise, positive = lower)
        
        float forward_velocity = -left_y;    // Negate: joy is negative for forward
        float strafe_velocity = left_x;      // Left stick X for strafing
        float rotation_velocity = right_x;   // Right stick X for rotation
        float height_change = -right_y;      // Negate: joy is negative for raise
        
        // Apply speed multiplier to movement
        float move_time_ms = BASE_MOVE_TIME / m_speed_multiplier;
        int move_time = static_cast<int>(move_time_ms);
        
        // Ensure minimum movement time
        move_time = std::max(move_time, 50);
        
        // =====================================================
        // SIMPLE BODY SWAY MODE (no walking gait yet)
        // All legs move together to shift body position
        // This is safe for initial testing
        // =====================================================
        
        // Calculate foot position changes in LEG-LOCAL frame
        // x = forward/back movement of foot (when body moves forward, feet move back)
        // y = lateral shift (for strafing)  
        // z = height change
        
        // Scaling factors (safe, small movements)
        const double MAX_X_DISPLACEMENT = 0.03;  // +/- 3cm forward/back
        const double MAX_Y_DISPLACEMENT = 0.02;  // +/- 2cm lateral
        const double MAX_Z_DISPLACEMENT = 0.03;  // +/- 3cm height
        
        // Standing position (neutral)
        const double STAND_X = 0.0;
        const double STAND_Y = 0.10;
        const double STAND_Z = -0.20;
        
        // Calculate desired positions based on joystick input
        // Body moves opposite to foot displacement (push feet back to move body forward)
        double target_x = STAND_X - (forward_velocity * MAX_X_DISPLACEMENT);
        double target_y = STAND_Y + (strafe_velocity * MAX_Y_DISPLACEMENT);
        double target_z = STAND_Z - (height_change * MAX_Z_DISPLACEMENT);
        
        // Clamp to safe ranges
        target_x = std::clamp(target_x, -0.05, 0.05);
        target_y = std::clamp(target_y, 0.05, 0.15);
        target_z = std::clamp(target_z, -0.25, -0.12);
        
        // Move all legs to same position (synchronized body sway)
        for (auto& leg_pair : m_legs) {
            int leg_id = leg_pair.first;
            auto leg = leg_pair.second;
            
            if (!leg->move(target_x, target_y, target_z, move_time, false)) {
                RCLCPP_WARN(this->get_logger(), "Failed to move leg %d to (%.3f, %.3f, %.3f)",
                    leg_id, target_x, target_y, target_z);
            }
        }
        
        // Update tracking
        m_foot_x = target_x;
        m_foot_y = target_y;
        m_foot_z = target_z;
    }
    
    void move_to_stand()
    {
        int move_time = 1500;  // 1.5 seconds for posture change
        
        RCLCPP_INFO(this->get_logger(), "Moving to STAND position...");
        
        // CORRECT leg-local coordinates for standing:
        // x = 0    (foot directly under hip in forward direction)
        // y = 0.10 (foot slightly outward for stability)
        // z = -0.20 (foot 20cm below hip - NEGATIVE because down)
        // 
        // These values were validated in servo_simulation_final.py
        // All servos stay within 0-1000 range
        
        const double stand_x = 0.0;
        const double stand_y = 0.10;
        const double stand_z = -0.20;
        
        for (auto& leg_pair : m_legs) {
            int leg_id = leg_pair.first;
            auto leg = leg_pair.second;
            
            if (!leg->move(stand_x, stand_y, stand_z, move_time, false)) {
                RCLCPP_WARN(this->get_logger(), "Failed to move leg %d to stand position", leg_id);
            }
        }
        
        // Store current foot positions for gait tracking
        m_foot_x = stand_x;
        m_foot_y = stand_y;
        m_foot_z = stand_z;
        
        RCLCPP_INFO(this->get_logger(), "Robot standing (x=%.2f, y=%.2f, z=%.2f)", 
                    stand_x, stand_y, stand_z);
    }
    
    void move_to_sit()
    {
        int move_time = 1500;  // 1.5 seconds for posture change
        
        RCLCPP_INFO(this->get_logger(), "Moving to SIT position...");
        
        // CORRECT leg-local coordinates for sitting:
        // x = 0     (foot directly under hip)
        // y = 0.10  (slightly outward)
        // z = -0.12 (foot 12cm below hip - less extension than stand)
        
        const double sit_x = 0.0;
        const double sit_y = 0.10;
        const double sit_z = -0.12;
        
        for (auto& leg_pair : m_legs) {
            int leg_id = leg_pair.first;
            auto leg = leg_pair.second;
            
            if (!leg->move(sit_x, sit_y, sit_z, move_time, false)) {
                RCLCPP_WARN(this->get_logger(), "Failed to move leg %d to sit position", leg_id);
            }
        }
        
        m_foot_x = sit_x;
        m_foot_y = sit_y;
        m_foot_z = sit_z;
        
        RCLCPP_INFO(this->get_logger(), "Robot sitting (x=%.2f, y=%.2f, z=%.2f)",
                    sit_x, sit_y, sit_z);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<RemoteControllerImpl>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Fatal error: %s", e.what());
        return -1;
    }
    
    rclcpp::shutdown();
    return 0;
}
