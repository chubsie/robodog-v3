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
#include "Button.h"

void Button::check(uint8_t buttons)
{
    bool on = buttons & m_bitId;

    if (on != m_on) {
        m_on = on;
        if (m_on) {
            m_holdStart = rclcpp::Clock().now();
        }
        else if (!m_clicked) {
            if (rclcpp::Clock().now() - m_lastClicked < rclcpp::Duration(1, 0)) m_doubleClicked = true;
            m_clicked = true;
            m_lastClicked = rclcpp::Clock().now();
        }
    }
}

bool Button::holded()
{
    if (m_on &&
        rclcpp::Clock().now()- m_holdStart > rclcpp::Duration(1, 0)) return true;
    return false;
}

bool Button::clicked()
{
    bool ret = m_clicked;
    m_clicked = m_doubleClicked = false;
    return ret;
}

bool Button::double_clicked()
{
    bool ret = m_doubleClicked;
    m_doubleClicked = false;
    return ret;
}
