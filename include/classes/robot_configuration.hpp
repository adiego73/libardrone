#ifndef ROBOTCONFIGURATION_H
#define ROBOTCONFIGURATION_H

#include <exception>
#include <iostream>
#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include "config/configuration.hpp"

namespace robot
{

class RobotConfiguration
{
    public:
        RobotConfiguration(std::string path);
        RobotConfig get();
        
        ~RobotConfiguration();

    private:
        std::string path;
        RobotConfig configuration;
        
        void parse();
};


}
#endif // ROBOTCONFIGURATION_H
