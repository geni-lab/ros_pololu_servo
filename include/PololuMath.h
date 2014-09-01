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
        static double to_pulse(double radians, Motor motor);
        static double to_radians(double pulse, Motor motor);
};
