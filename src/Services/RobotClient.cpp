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
#include <iostream>
#include <fstream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
// #include "robodog/srv/gcode_service.hpp"

// bool SendGcode(std::string gcode)
// {
//     auto node = rclcpp::Node::make_shared("gcode_client");
//     auto client = node->create_client<robodog::srv::GcodeService>("gcode");

//     auto request = std::make_shared<robodog::srv::GcodeService::Request>();
//     request->gcode = gcode;

//     while (!client->wait_for_service(std::chrono::seconds(1))) {
//         if (!rclcpp::ok()) {
//             RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
//             return false;
//         }
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
//     }

//     auto result = client->async_send_request(request);
//     // Wait for the result.
//     if (rclcpp::spin_until_future_complete(node, result) ==
//         rclcpp::FutureReturnCode::SUCCESS)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gcode sent");
//     } else {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service gcode");
//     }

//     return true;
// }

// bool SendGcodeFromFile(const std::string& path)
// {
//     std::ifstream file(path);
//     if (!file.is_open()) {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "SendGcodeFromFile(): Failed to open file: %s", path.c_str());
//         return false;
//     }

//     std::stringstream strStream;
//     strStream << file.rdbuf();

//     return SendGcode(strStream.str());
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // SendGcodeFromFile("GCODE/test.gcode");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "robodog_client is running, but Gcode sending is disabled.");


    rclcpp::shutdown();
    return 0;
}
