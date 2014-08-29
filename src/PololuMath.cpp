
#include "PololuMath.h"

struct Calibration;
struct Motor;
double PololuMath::EPSILON = 0.001;


double PololuMath::to_motor_pos(double value, Motor motor)
{
    return (value + motor.init) * motor.direction;
}

double PololuMath::to_joint_pos(double value, Motor motor)
{
    return (value + motor.init) * motor.direction;
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

double PololuMath::angle_to_pulse(double angle, Calibration calibration)
{
    return interpolate(angle, (double)calibration.min_angle, (double)calibration.max_angle, (double)calibration.min_pulse, (double)calibration.max_pulse) * 4.0;
}

double PololuMath::pulse_to_angle(int pulse, Calibration calibration)
{
    return interpolate((double)pulse, (double)calibration.min_pulse * 4.0, (double)calibration.max_pulse * 4.0, (double)calibration.min_angle, (double)calibration.max_angle);
}