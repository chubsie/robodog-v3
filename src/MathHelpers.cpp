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
#include "MathHelpers.h"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

TimeFunc::TimeFunc(double from, double to, double sec, double exponent) :
        m_from(from), m_to(to), m_exponent(exponent), m_duration(rclcpp::Duration::from_seconds(sec))
{
    reset();
}

void TimeFunc::reset()
{
    m_curr = m_from;
    m_start = rclcpp::Clock().now();
}

double TimeFunc::get()
{
    rclcpp::Duration passed = rclcpp::Clock().now() - m_start;
    double k = passed.seconds() / m_duration.seconds();
    if (k >= 1) return m_to;

    // Make it exponential
    //k = k * k;
    k = pow(k, m_exponent);

    return m_from + (k * (m_to - m_from));
}

double constrain(double x, double min_x, double max_x)
{
    if (x < min_x) x = min_x;
    if (x > max_x) x = max_x;
    return x;
}
