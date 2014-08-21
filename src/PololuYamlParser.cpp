#include "PololuYamlParser.h"
#include "PololuMath.h"

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

            // Calculate direction
            int min_sgn = sgn(motor.min);
            int max_sgn = sgn(motor.max);

            // Check min / max signs
            if(min_sgn == max_sgn || max_sgn < 0 || 0 < min_sgn)
            {
                ROS_ERROR("motor %s, min (%f) and max (%f) cannot have the same sign, max cannot be less than 0 and min cannot be greater than 0", motor.name.c_str(), to_degrees(motor.min), to_degrees(motor.max));
                success = false;
            }

            //
            double min_range = fabs(-M_PI/2 - motor.init);
            double max_range = fabs(M_PI/2 - motor.init);

            if(fabs(motor.min) > min_range)
            {
                ROS_ERROR("motor %s, -ve range (%f) cannot be less than %f", motor.name.c_str(), to_degrees(motor.min), to_degrees(-min_range));
                success = false;
            }
            else if(motor.max > max_range)
            {
                ROS_ERROR("motor %s, +ve range (%f) cannot be greater than %f", motor.name.c_str(), to_degrees(motor.max), to_degrees(max_range));
                success = false;
            }

            ROS_INFO("Added motor (id: %d/%d, name: %s, min: %f, init: %f, max: %f)", motor.pololu_id, motor.motor_id, motor.name.c_str(), to_degrees(motor.min), to_degrees(motor.init), to_degrees(motor.max));
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
            motor.min = from_degrees(node["min"].as<double>());
            motor.init = from_degrees(node["init"].as<double>());
            motor.max = from_degrees(node["max"].as<double>());
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





