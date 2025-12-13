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
#include "IK.h"
#include "Utils.h"
#include "Factory.h"
#include "Servos/ServoController.h"
#include "BaseState.h"
#include "rclcpp/rclcpp.hpp"
#include "io_context/io_context.hpp"

#include <math.h>

using drivers::common::IoContext;

const double kDegToServo_v3 = 1.0 / 0.24;
const double kWheelRatio_v3 = 2.0;
const int    kMaxTemp_v3    = 65;

class RobotV3 : public Robot
{
public:
    /**
     * CORRECTED angle_to_servo_pos() - Validated via servo_simulation_final.py
     * 
     * KEY FIXES:
     * 1. Servos grouped by FUNCTION (from PDF), not by ID number
     * 2. Hip/Knee servos use 90° as neutral angle (not 0°)
     * 3. Direction determined by PDF max/min values
     * 
     * SERVO CLASSIFICATION (from Leg_HW_Config.pdf):
     * | Leg | Abductor | Hip_Inner | Hip_Outer | Knee_Inner | Knee_Outer |
     * |-----|----------|-----------|-----------|------------|------------|
     * | FR  | 6        | 20        | 8         | 19         | 17         |
     * | FL  | 14       | 3         | 15        | 9          | 10         |
     * | BL  | 1        | 12        | 4         | 16         | 11         |
     * | BR  | 2        | 13        | 18        | 7          | 5          |
     */
    virtual bool angle_to_servo_pos(int servoId, double angle, int& pos, bool silent) override
    {
        if (std::isnan(angle)) {
            if (!silent) RCLCPP_ERROR(rclcpp::get_logger("RobotV3"), "angle_to_servo_pos(servoId=%d): angle %f is NAN", servoId, angle);
            return false;
        }

        // Convert angle to degrees for formula
        double angleDeg = IK::toDeg(angle);

        switch (servoId) {
            // ========== ABDUCTORS ==========
            // Neutral angle = 0°, uses wheel ratio
            case 1:  // BL Abductor - negative direction
                pos = 500 - angleDeg * kDegToServo_v3 * kWheelRatio_v3;
                break;
            case 2:  // BR Abductor - positive direction
                pos = 500 + angleDeg * kDegToServo_v3 * kWheelRatio_v3;
                break;
            case 6:  // FR Abductor - positive direction
                pos = 500 + angleDeg * kDegToServo_v3 * kWheelRatio_v3;
                break;
            case 14: // FL Abductor - negative direction
                pos = 500 - angleDeg * kDegToServo_v3 * kWheelRatio_v3;
                break;

            // ========== HIP_INNER ==========
            // Neutral angle = 90°, no wheel ratio
            case 3:  // FL Hip_Inner - positive direction
                pos = 500 + (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 12: // BL Hip_Inner - positive direction
                pos = 500 + (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 13: // BR Hip_Inner - negative direction
                pos = 500 - (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 20: // FR Hip_Inner - negative direction
                pos = 500 - (angleDeg - 90.0) * kDegToServo_v3;
                break;

            // ========== HIP_OUTER ==========
            // Neutral angle = 90°, no wheel ratio
            case 4:  // BL Hip_Outer - negative direction
                pos = 500 - (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 8:  // FR Hip_Outer - positive direction
                pos = 500 + (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 15: // FL Hip_Outer - negative direction
                pos = 500 - (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 18: // BR Hip_Outer - positive direction
                pos = 500 + (angleDeg - 90.0) * kDegToServo_v3;
                break;

            // ========== KNEE_INNER ==========
            // Neutral angle = 90°, no wheel ratio
            case 7:  // BR Knee_Inner - negative direction
                pos = 500 - (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 9:  // FL Knee_Inner - positive direction
                pos = 500 + (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 16: // BL Knee_Inner - positive direction
                pos = 500 + (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 19: // FR Knee_Inner - negative direction
                pos = 500 - (angleDeg - 90.0) * kDegToServo_v3;
                break;

            // ========== KNEE_OUTER ==========
            // Neutral angle = 90°, no wheel ratio
            case 5:  // BR Knee_Outer - positive direction
                pos = 500 + (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 10: // FL Knee_Outer - negative direction
                pos = 500 - (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 11: // BL Knee_Outer - negative direction
                pos = 500 - (angleDeg - 90.0) * kDegToServo_v3;
                break;
            case 17: // FR Knee_Outer - positive direction
                pos = 500 + (angleDeg - 90.0) * kDegToServo_v3;
                break;

            default:
                RCLCPP_ERROR(rclcpp::get_logger("RobotV3"), "angle_to_servo_pos(): Unknown servo id %d", servoId);
                return false;
        }

        // Range check
        if (pos < 0) {
            if (!silent) RCLCPP_ERROR(rclcpp::get_logger("RobotV3"), "Servo %d out of range (%d), adjust to 0", servoId, pos);
            pos = 0;
            return false;
        }
        else if (pos > 1000) {
            if (!silent) RCLCPP_ERROR(rclcpp::get_logger("RobotV3"), "Servo %d out of range (%d), adjust to 1000", servoId, pos);
            pos = 1000;
            return false;
        }

        return true;
    }

    virtual ServoController* GetServoCtrl() override { return m_servoCtrl.get(); }
    virtual GCodeController* GetGcodeCtrl() override { return m_gcodeCtrl.get(); }

    virtual int GetVer() override { return 3; }

public:
    std::shared_ptr<ServoController> m_servoCtrl;
    std::shared_ptr<GCodeController> m_gcodeCtrl;
};

namespace Factory
{
    bool CreateRobotV3(std::shared_ptr<Robot>& robot)
    {
        // Create IoContext for serial communication
        auto io_ctx = std::make_shared<IoContext>();
        
        auto lx16a = std::make_shared<LX16a>(io_ctx);
        
        // Open serial port - configure this path based on your system
        // On Jetson with USB-to-Serial adapter, this is typically /dev/ttyUSB0 or /dev/ttyACM0
        // You can check with: ls -la /dev/tty* | grep USB
        if (!lx16a->open("/dev/ttyUSB0")) {
            RCLCPP_WARN(rclcpp::get_logger("Factory"), "Failed to open /dev/ttyUSB0, attempting /dev/ttyACM0");
            if (!lx16a->open("/dev/ttyACM0")) {
                RCLCPP_ERROR(rclcpp::get_logger("Factory"), "Failed to open servo serial port on either /dev/ttyUSB0 or /dev/ttyACM0");
                return false;
            }
        }

        auto robotV3 = std::make_shared<RobotV3>();

        // Create servo with offsets
        auto s1_1 = Factory::CreateLX16a(1, lx16a, 0);
        auto s1_2 = Factory::CreateLX16a(2, lx16a, -60);
        auto s1_3 = Factory::CreateLX16a(3, lx16a, -20);
        auto s1_4 = Factory::CreateLX16a(4, lx16a, -10);
        auto s1_5 = Factory::CreateLX16a(5, lx16a, 0);

        auto s2_1 = Factory::CreateLX16a( 6, lx16a, 0);
        auto s2_2 = Factory::CreateLX16a( 7, lx16a, 10);
        auto s2_3 = Factory::CreateLX16a( 8, lx16a, -10);
        auto s2_4 = Factory::CreateLX16a( 9, lx16a, -1);
        auto s2_5 = Factory::CreateLX16a(10, lx16a, -3);

        auto s3_1 = Factory::CreateLX16a(11, lx16a, 0);
        auto s3_2 = Factory::CreateLX16a(12, lx16a, 30);
        auto s3_3 = Factory::CreateLX16a(13, lx16a, 0);
        auto s3_4 = Factory::CreateLX16a(14, lx16a, 0);
        auto s3_5 = Factory::CreateLX16a(15, lx16a, 10);

        auto s4_1 = Factory::CreateLX16a(16, lx16a, 0);
        auto s4_2 = Factory::CreateLX16a(17, lx16a, 0);
        auto s4_3 = Factory::CreateLX16a(18, lx16a, -10);
        auto s4_4 = Factory::CreateLX16a(19, lx16a, -20);
        auto s4_5 = Factory::CreateLX16a(20, lx16a, 25);

        std::vector<std::shared_ptr<Servo>> servos;

        servos.push_back(s1_1);
        servos.push_back(s1_2);
        servos.push_back(s1_3);
        servos.push_back(s1_4);
        servos.push_back(s1_5);

        servos.push_back(s2_1);
        servos.push_back(s2_2);
        servos.push_back(s2_3);
        servos.push_back(s2_4);
        servos.push_back(s2_5);

        servos.push_back(s3_1);
        servos.push_back(s3_2);
        servos.push_back(s3_3);
        servos.push_back(s3_4);
        servos.push_back(s3_5);

        servos.push_back(s4_1);
        servos.push_back(s4_2);
        servos.push_back(s4_3);
        servos.push_back(s4_4);
        servos.push_back(s4_5);

        for (auto& s : servos) {
            s->set_max_temp(kMaxTemp_v3);
        }

        robotV3->m_servoCtrl = std::make_shared<ServoController>();
        robotV3->m_servoCtrl->AddServoGroup(servos);

        std::map<uint, std::shared_ptr<Joint>> leg1_joints; // Front Right
        leg1_joints[1] = Factory::CreateJointSingle(s2_1,       robotV3); // 6
        leg1_joints[2] = Factory::CreateJointDouble(s4_5, s2_3, robotV3); // 20, 8
        leg1_joints[3] = Factory::CreateJointDouble(s4_4, s4_2, robotV3); // 19, 17

        std::map<uint, std::shared_ptr<Joint>> leg2_joints; // Front Left
        leg2_joints[1] = Factory::CreateJointSingle(s3_4,       robotV3); // 14
        leg2_joints[2] = Factory::CreateJointDouble(s1_3, s3_5, robotV3); // 3, 15
        leg2_joints[3] = Factory::CreateJointDouble(s2_4, s2_5, robotV3); // 9, 10

        std::map<uint, std::shared_ptr<Joint>> leg3_joints; // Back Left
        leg3_joints[1] = Factory::CreateJointSingle(s1_1,       robotV3); // 1
        leg3_joints[2] = Factory::CreateJointDouble(s3_2, s1_4, robotV3); // 12, 4
        leg3_joints[3] = Factory::CreateJointDouble(s4_1, s3_1, robotV3); // 16, 11

        std::map<uint, std::shared_ptr<Joint>> leg4_joints; // Back Right
        leg4_joints[1] = Factory::CreateJointSingle(s1_2,       robotV3); // 2
        leg4_joints[2] = Factory::CreateJointDouble(s3_3, s4_3, robotV3); // 13, 18
        leg4_joints[3] = Factory::CreateJointDouble(s2_2, s1_5, robotV3); // 7, 5

        auto leg1 = Factory::CreateLeg(1, leg1_joints, false);
        auto leg2 = Factory::CreateLeg(2, leg2_joints, false);
        auto leg3 = Factory::CreateLeg(3, leg3_joints, false);
        auto leg4 = Factory::CreateLeg(4, leg4_joints, false);

        leg1->set_geometry(0.029, 0.165, 0.180); // + leg foot ball (20mm radius)
        leg2->set_geometry(0.029, 0.165, 0.180);
        leg3->set_geometry(0.029, 0.165, 0.180);
        leg4->set_geometry(0.029, 0.165, 0.180);

        // In base frame
        leg1->set_leg_location( 0.209, -0.115, 0);
        leg2->set_leg_location( 0.209,  0.115, 0);
        leg3->set_leg_location(-0.209, -0.099, 0);
        leg4->set_leg_location(-0.209,  0.099, 0);

        std::map<uint8_t, std::shared_ptr<Leg>> legs;
        legs[leg1->getId()] = leg1;
        legs[leg2->getId()] = leg2;
        legs[leg3->getId()] = leg3;
        legs[leg4->getId()] = leg4;

        for (auto& l : legs) {
            l.second->init_with_pos(0, 0.1, 0);
        }
        utils::delay_ms(300);

        robotV3->m_gcodeCtrl = Factory::CreateGCodeController(legs, robotV3);

        robot = robotV3;

        return true;
    }
}
