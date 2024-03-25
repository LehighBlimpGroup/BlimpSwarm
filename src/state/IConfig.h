


#pragma once

template<typename ConfigType>
class IConfig {
public:
    virtual ~IConfig() {}
    virtual void loadConfiguration() = 0; // To be implemented by subclasses
    virtual const ConfigType& getConfiguration() const = 0; // To be implemented by subclasses
};
