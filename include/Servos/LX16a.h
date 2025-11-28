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
#pragma once

#include <string>
#include <memory>
#include <vector>
#include <cstdint>

#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"
#include "io_context/io_context.hpp"

using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPort;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::StopBits;

class LX16a
{
public:
    // Default constructor for unit tests and backward compatibility
    LX16a() 
        : m_unitTestRun(false), m_io_ctx(nullptr), m_serial_driver(nullptr), m_port(nullptr) {}
    
    // Constructor with IoContext for actual serial communication
    LX16a(std::shared_ptr<IoContext> io_ctx) 
        : m_unitTestRun(false), m_io_ctx(io_ctx), m_serial_driver(nullptr), m_port(nullptr) {}
    
    virtual ~LX16a() {
        if (m_port && m_port->is_open()) {
            m_port->close();
        }
    }

    bool open(const std::string port);

    bool move(uint8_t id, uint16_t angle, uint16_t time = 0);
    bool set_offset(uint8_t id, uint8_t angle);

    bool motor_off(uint8_t id);

    bool get_pos(uint8_t id, int16_t& pos);

    bool get_id(uint8_t& id);

    bool get_temp(uint8_t id, uint8_t& temp);
    bool get_vin(uint8_t id, uint16_t& vin);

    enum LED_ERR {
        kNoErr              = 0,
        kOverTemp           = 1,
        kOverVolt           = 2,
        kOverTempAndVolt    = 3,
        kLockedRotor        = 4,
        kOverTempAndStalled = 5,
        kOverVoltAndStalled = 6
    };
    bool get_led_err(uint8_t id, uint8_t& err);

    bool set_max_temp(uint8_t id, uint8_t temp);

private:
    bool send_packet(uint8_t id, int8_t cmd, const uint8_t* params = nullptr, size_t param_cnt = 0);
    bool read_packet(uint8_t id, uint8_t cmd, uint8_t *params = nullptr, size_t param_cnt = 0);

    bool read_packet_with_retries(uint8_t id, uint8_t cmd, uint8_t *params, size_t param_cnt, int retries);

    uint8_t read_byte();

    std::shared_ptr<SerialPort> m_port;
    bool m_unitTestRun;
    std::shared_ptr<IoContext> m_io_ctx;
    std::shared_ptr<SerialDriver> m_serial_driver;
};
