/**
 * @file IConfig.h
 * @author Swarms Lab
 * @brief Generic parent class for loading and getting configuration parameters for the robot states
 * @version 0.1
 * @date 2024-01-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

template<typename ConfigType>
class IConfig {
    public:
        virtual ~IConfig() {}
        virtual void loadConfiguration() = 0;
        virtual const ConfigType& getConfiguration() const = 0;
};
