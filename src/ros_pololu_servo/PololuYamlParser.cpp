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

#include <ros_pololu_servo/PololuYamlParser.h>
#include <ros_pololu_servo/PololuMath.h>

using namespace std;



template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

bool PololuYamlParser::parse(string directory, map<string, Motor> &motors)
{
    YAML::Node motors_config = YAML::LoadFile(directory);
    bool success = true;

    try
    {
        for(YAML::const_iterator it=motors_config.begin(); it!=motors_config.end(); ++it)
        {
            Motor motor = it->second.as<Motor>();
            motor.name = it->first.as<string>();
            motors[motor.name] = motor;

            if(!(motor.min < motor.max))
            {
                ROS_ERROR("motor %s: min (%f) must be less than motor max (%f)", motor.name.c_str(), motor.min, motor.max);
                success = false;
            }

            if(motor.min >= motor.init)
            {
                ROS_ERROR("motor %s: min (%f) must be less than init (%f). Attempting to fail gracefully by clamping.", motor.name.c_str(), motor.min, motor.init);
                motor.init = motor.min + 1.0;
            }

            if(motor.init >= motor.max)
            {
                ROS_ERROR("motor %s: init (%f) must be less than max (%f). Attempting to fail gracefully by clamping. ", motor.name.c_str(), motor.init, motor.max);
                motor.init = motor.max - 1.0;
            }

            ROS_INFO("Added motor (id: %d/%d, name: %s, min: %f, init: %f, max: %f)", motor.pololu_id, motor.motor_id, motor.name.c_str(), motor.min, motor.init, motor.max);
        }
    }
    catch(YAML::ParserException& e)
    {
        ROS_ERROR("Problem parsing pololu_motors_yaml: %s", e.what());
        success = false;
    }

    return success;
}

namespace YAML
{
    template<>
    struct convert<Calibration>
    {
        static bool decode(const Node& node, Calibration& calibration)
        {
            if(!node.IsMap())
            {
                ROS_ERROR("Error, calibration does not contain key value pairs, i.e. min_pulse, min_angle, max_pulse and max_angle");
                return false;
            }
            else if(!node["min_pulse"])
            {
                ROS_ERROR("Error, calibration min_pulse not specified");
                return false;
            }
            else if(!node["min_angle"])
            {
                ROS_ERROR("Error, calibration min_angle not specified");
                return false;
            }
            else if(!node["max_pulse"])
            {
                ROS_ERROR("Error, calibration max_pulse not specified");
                return false;
            }
            else if(!node["max_angle"])
            {
                ROS_ERROR("Error, calibration max_angle not specified");
                return false;
            }

            calibration.min_pulse = node["min_pulse"].as<int>();
            calibration.min_angle = from_degrees(node["min_angle"].as<double>());
            calibration.max_pulse = node["max_pulse"].as<int>();
            calibration.max_angle = from_degrees(node["max_angle"].as<double>());

            return true;
        }
    };


    template<>
    struct convert<Motor>
    {
        static bool decode(const Node& node, Motor& motor)
        {
            if(!node.IsMap())
            {
                ROS_ERROR("Error, motor does not contain key value pairs, i.e. pololu_id, motor_id, min, init, max, calibration and reverse");
                return false;
            }
            else if(!node["pololu_id"])
            {
                ROS_ERROR("Error, motor pololu_id not specified");
                return false;
            }
            else if(!node["motor_id"])
            {
                ROS_ERROR("Error, motor motor_id not specified");
                return false;
            }
            else if(!node["min"])
            {
                ROS_ERROR("Error, motor min not specified");
                return false;
            }
            else if(!node["init"])
            {
                ROS_ERROR("Error, motor init not specified");
                return false;
            }
            else if(!node["max"])
            {
                ROS_ERROR("Error, motor max not specified");
                return false;
            }
            else if(!node["calibration"])
            {
                ROS_ERROR("Error, calibration not specified");
                return false;
            }
            else if(!node["reverse"])
            {
                ROS_ERROR("Error, motor reverse not specified");
                return false;
            }

            motor.pololu_id = node["pololu_id"].as<int>();
            motor.motor_id = node["motor_id"].as<int>();
            motor.min = node["min"].as<double>();
            motor.init = node["init"].as<double>();
            motor.max = node["max"].as<double>();
            motor.calibration = node["calibration"].as<Calibration>();
            motor.direction = 1.0;

            if(node["reverse"].as<bool>())
            {
                motor.direction = -1.0;
            }

            return true;
        }
    };
}





