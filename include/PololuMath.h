#pragma once

#include <ros/ros.h>
#include "Motor.h"
#include "Calibration.h"
#include <angles/angles.h>

class PololuMath
{
    public:
        static double EPSILON;
        static bool are_same(double a, double b);
        static double interpolate(double value, double old_min, double old_max, double new_min, double new_max);
        static double angle_to_pulse(double angle, Calibration calibration);
        static double pulse_to_angle(int pulse, Calibration calibration);
        static double to_motor_pos(double value, Motor motor);
        static double to_joint_pos(double value, Motor motor);
};
