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
#include <chrono>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#include "Utils.h"
#include "Servos/LX16a.h"
#include "rclcpp/rclcpp.hpp"

//#define DEBUG_LOG

using namespace std;

bool LX16a::open(const std::string port)
{
    if (port.length() == 0) {
        m_unitTestRun = true;
        return true;    // For unit tests
    }

    try {
        if (!m_io_ctx) {
            RCLCPP_ERROR(rclcpp::get_logger("LX16a"), "IoContext not initialized");
            return false;
        }

        // Create serial driver and port configuration
        m_serial_driver = std::make_shared<SerialDriver>(*m_io_ctx);
        
        // LX16a servos typically use 115200 baud rate
        SerialPortConfig config(
            115200,
            FlowControl::NONE,
            Parity::NONE,
            StopBits::ONE
        );

        m_serial_driver->init_port(port, config);
        m_port = m_serial_driver->port();

        if (!m_port) {
            RCLCPP_ERROR(rclcpp::get_logger("LX16a"), "Failed to initialize serial port");
            return false;
        }

        m_port->open();
        
        if (!m_port->is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("LX16a"), "Failed to open serial port: %s", port.c_str());
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("LX16a"), "Serial port opened successfully: %s", port.c_str());
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("LX16a"), "Exception opening serial port: %s", e.what());
        return false;
    }
}

uint8_t LX16a::read_byte()
{
    if (!m_port || !m_port->is_open()) return m_unitTestRun;

    // Pre-allocate buffer - transport_drivers requires pre-sized vector
    std::vector<uint8_t> buffer(128);
    try {
        size_t bytes_read = m_port->receive(buffer);
        if (bytes_read > 0) {
            return buffer[0];
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("LX16a"), "Error reading byte: %s", e.what());
    }
    return 0;
}

bool LX16a::send_packet(uint8_t id, int8_t cmd, const uint8_t* params, size_t param_cnt)
{
   if (!m_port || !m_port->is_open()) return m_unitTestRun;

   if (param_cnt > 4)
        return false;

    int buflen = 6 + param_cnt;
    std::vector<uint8_t> buf(buflen);
    buf[0] = 0x55;
    buf[1] = 0x55;
    buf[2] = id;
    buf[3] = buflen-3;
    buf[4] = cmd;

    for (size_t i = 0; i < param_cnt; i++)
        buf[5+i] = params[i];

    uint8_t cksum = 0;
    for (int i = 2; i < buflen-1; i++)
        cksum += buf[i];

    buf[buflen-1] = ~cksum;

#ifdef DEBUG_LOG
        printf("SEND: ");
        for (int i = 0; i < buflen; i++)
            printf(" %02X", buf[i]);
        printf("\n");
#endif

    try {
        m_port->send(buf);
        // Give servo time to process and respond (LX16a needs ~1-5ms, plus USB latency)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("LX16a"), "Error sending packet: %s", e.what());
        return false;
    }
}

bool LX16a::read_packet(uint8_t id, uint8_t cmd, uint8_t *params, size_t param_cnt)
{
    if (!m_port || !m_port->is_open()) return m_unitTestRun;

#ifdef DEBUG_LOG
    printf("RECV: ");
#endif

    int got = 0;
    int len = 7; // Minimum length
    uint8_t sum = 0;
    bool bytes_received = false;
    bool synced = false;  // Track if we've found the 0x55 0x55 header
    int sync_count = 0;   // Count consecutive 0x55 bytes

    // Timeout after 150ms
    auto timeout = std::chrono::high_resolution_clock::now() + std::chrono::milliseconds(150);
    
    // Use a larger buffer to get multiple bytes at once
    std::vector<uint8_t> recv_buf(128);
    int buf_pos = 0;
    int buf_len = 0;

    while (std::chrono::high_resolution_clock::now() < timeout) {
        uint8_t byte;
        
        // If we've consumed all buffered data, try to read more
        if (buf_pos >= buf_len) {
            try {
                buf_len = m_port->receive(recv_buf);
                buf_pos = 0;
                
                if (buf_len == 0) {
                    // No data, sleep and retry
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
            } catch (const std::exception& e) {
                if (std::string(e.what()).find("Interrupted") != std::string::npos) {
                    // Retry on interrupt
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
                    continue;
                }
                // For other errors, just sleep and try again
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
        }
        
        if (buf_pos < buf_len) {
            byte = recv_buf[buf_pos++];
            bytes_received = true;
            
            // If not synced yet, look for 0x55 0x55 header
            if (!synced) {
                if (byte == 0x55) {
                    sync_count++;
                    if (sync_count == 2) {
                        synced = true;
                        got = 2;  // We've received the two sync bytes
#ifdef DEBUG_LOG
                        printf("55 55 ");
#endif
                    }
                } else {
                    sync_count = 0;  // Reset if we see a non-0x55 byte
                }
                continue;
            }
            
            // Once synced, process packet normally
            if (got >= len) break;  // Packet complete
#ifdef DEBUG_LOG
            printf("%02X ", byte);
#endif
            
            switch (got) {
                case 2:
                    if (byte != id && id != 0xfe) {
                        RCLCPP_WARN(rclcpp::get_logger("LX16a"), "read_packet(id=%d): Servo ID mismatch: expected 0x%02X, got 0x%02X", id, id, byte);
                        return false;
                    }
                    break;
                case 3:
                    if (byte < 3 || byte > 7) {
                        RCLCPP_WARN(rclcpp::get_logger("LX16a"), "read_packet(id=%d): Invalid length byte: 0x%02X (must be 3-7)", id, byte);
                        return false;
                    }
                    len = byte + 3;
                    if (len > (int)param_cnt + 6) {
                        RCLCPP_WARN(rclcpp::get_logger("LX16a"), "read_packet(id=%d): Length exceeds buffer: len=%d, buffer_size=%zu", id, len, param_cnt + 6);
                        return false;
                    }
                    break;
                case 4:
                    if (byte != cmd) {
                        RCLCPP_WARN(rclcpp::get_logger("LX16a"), "read_packet(id=%d): Command mismatch: expected 0x%02X, got 0x%02X", id, cmd, byte);
                        return false;
                    }
                    break;
                default:
                    if (got == len - 1) {
                        if (byte == (uint8_t)~sum) {
#ifdef DEBUG_LOG
                            printf("\n");
#endif
                            return true;
                        } else {
                            RCLCPP_WARN(rclcpp::get_logger("LX16a"), "read_packet(id=%d): Checksum mismatch: expected 0x%02X, got 0x%02X", id, (uint8_t)~sum, byte);
                            return false;
                        }
                    }
                    if (got - 5 > (int)param_cnt) {
                        RCLCPP_WARN(rclcpp::get_logger("LX16a"), "read_packet(id=%d): Parameter index out of bounds", id);
                        return false;
                    }
                    params[got - 5] = byte;
            }

            if (got > 1)
                sum += byte;
            got++;
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    if (!bytes_received) {
        RCLCPP_WARN(rclcpp::get_logger("LX16a"), "read_packet(id=%d, cmd=0x%02X): No response received from servo (timeout after 150ms)", id, cmd);
    } else if (!synced) {
        RCLCPP_WARN(rclcpp::get_logger("LX16a"), "read_packet(id=%d, cmd=0x%02X): Failed to sync to packet header (timeout after 150ms)", id, cmd);
    }
    
    return false;
}

bool LX16a::read_packet_with_retries(uint8_t id, uint8_t cmd, uint8_t *params, size_t param_cnt, int retries)
{
    for (int i = 0; i < retries; ++i) {
        if (read_packet(id, cmd, params, param_cnt)) return true;
    }
    return false;
}

// 0..1000
bool LX16a::move(uint8_t id, uint16_t angle, uint16_t time)
{
    uint8_t params[4] = { (uint8_t)angle, (uint8_t)(angle>>8), (uint8_t)time, (uint8_t)(time>>8) };

    if (!send_packet(id, 1, params, sizeof(params)))
        return false;

    return true;
}

// -125...125
bool LX16a::set_offset(uint8_t id, uint8_t angle)
{
    uint8_t params[1] = { (uint8_t)angle };

    if (!send_packet(id, 17, params, sizeof(params)))
        return false;

    if (!send_packet(id, 18, params, 0))
        return false;

    utils::delay_ms(100);

    return true;
}

bool LX16a::motor_off(uint8_t id)
{
    uint8_t params[1] = { 0 };
    return send_packet(id, 31, params, sizeof(params));
}

bool LX16a::get_pos(uint8_t id, int16_t& pos)
{
    const int kMaxAttempts = 1;  // Single attempt first
    int i = 0;
    uint8_t params[2];
    for (; i < kMaxAttempts; ++i) {
        if (!send_packet(id, 28)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        if (!read_packet(id, 28, params, sizeof(params))) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        break;
    }

    if (i == kMaxAttempts) {
        RCLCPP_ERROR(rclcpp::get_logger("LX16a"), "get_pos: read_packet(id=%d) failed", id);
        return false;
    }

    pos = ( (int16_t)params[0] | ((int16_t)params[1]<<8) );
    return true;
}

bool LX16a::get_id(uint8_t& id)
{
    if (!send_packet(0xFE, 14))
        return false;

    uint8_t params[1];
    if (!read_packet(0xFE, 14, params, sizeof(params)))
        return false;

    id = params[0];
    return true;
}

bool LX16a::get_temp(uint8_t id, uint8_t& temp)
{
    if (!send_packet(id, 26))
        return false;

    uint8_t params[1];
    if (!read_packet(id, 26, params, sizeof(params)))
        return false;

    temp = params[0];
    return true;
}

bool LX16a::get_vin(uint8_t id, uint16_t& vin)
{
    if (!send_packet(id, 27))
        return false;

    uint8_t params[2];
    if (!read_packet(id, 27, params, sizeof(params)))
        return false;

    vin = ( (int16_t)params[0] | ((int16_t)params[1]<<8) );
    return true;
}

bool LX16a::get_led_err(uint8_t id, uint8_t& err)
{
    if (!send_packet(id, 36))
        return false;

    uint8_t params[1];
    if (!read_packet(id, 36, params, sizeof(params)))
        return false;

    err = params[0];
    return true;
}

bool LX16a::set_max_temp(uint8_t id, uint8_t temp)
{
    uint8_t params[1] = { (uint8_t)temp };

    if (!send_packet(id, 24, params, sizeof(params)))
        return false;

    utils::delay_ms(100);

    return true;
}
