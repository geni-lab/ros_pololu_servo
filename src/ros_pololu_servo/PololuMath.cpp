/* Copyright (c) 2014, OpenCog Foundation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the OpenCog Foundation nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros_pololu_servo/PololuMath.h>

struct Calibration;
struct Motor;
double PololuMath::EPSILON = 0.001;


double PololuMath::clamp(double value, double min, double max)
{
    if(value < min)
    {
        return min;
    }
    else if(value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}

double PololuMath::to_pulse(double radians, Motor motor)
{
    double range_pwm = motor.calibration.max_pulse - motor.calibration.min_pulse;
    double range_rads = motor.calibration.max_angle - motor.calibration.min_angle;
    double scale = range_pwm / range_rads;
    return (radians * motor.direction * scale) + motor.init;
}

double PololuMath::to_radians(double pulse, Motor motor)
{
    double range_pwm = motor.calibration.max_pulse - motor.calibration.min_pulse;
    double range_rads = motor.calibration.max_angle - motor.calibration.min_angle;
    double scale = range_pwm / range_rads;
    return (pulse - motor.init) / scale * motor.direction;
}

bool PololuMath::are_same(double a, double b)
{
    return fabs(a - b) < PololuMath::EPSILON;
}

double PololuMath::interpolate(double value, double old_min, double old_max, double new_min, double new_max)
{
    // Width of each range
    double old_range = old_max - old_min;
    double new_range = new_max - new_min;

    // Scale old value into range between 0 and 1
    double scaled_value = double(value - old_min) / double(old_range);

    // Convert the scaled value into the new range
    double new_val = new_min + (scaled_value * new_range);

    //printf("old_range: %f, new_range: %f, scaled_val: %f, new_val: %f", old_range, new_range, scaled_value, new_val);

    return new_val;
}
