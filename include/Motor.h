#pragma once
#include <string>
#include "Calibration.h"

struct Calibration;

struct Motor
{
    std::string name;
    int pololu_id;
    int motor_id;
    double init;
    double min;
    double max;
    double direction;
    Calibration calibration;
};