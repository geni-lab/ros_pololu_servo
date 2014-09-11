
#include "PololuMath.h"

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
