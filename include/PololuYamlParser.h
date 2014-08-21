#pragma once

#include <ros/ros.h>
#include <angles/angles.h>
#include "yaml-cpp/yaml.h"
#include "Calibration.h"
#include "Motor.h"
#include "PololuMath.h"

using namespace std;
using namespace angles;


class PololuYamlParser
{
    public:
        static bool parse(std::string directory, map<string, Motor> &motors);

};
