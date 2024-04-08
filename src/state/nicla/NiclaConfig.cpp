
#include "state/nicla/NiclaConfig.h"

// Initialize the static instance pointer of NiclaConfig to nullptr
NiclaConfig* NiclaConfig::instance = nullptr;

// Private constructor definition
NiclaConfig::NiclaConfig() {
    loadConfiguration();
}

// Implementation of loadConfiguration
void NiclaConfig::loadConfiguration() {
    Preferences preferences;
    preferences.begin("params", true);

    // examples
    // configData.roll = preferences.getBool("roll", false);
    // configData.sensitivity = preferences.getInt("sensitivity", 10);
    
    configData.y_thresh = preferences.getFloat("y_thresh", 0.65);
    configData.y_strength = preferences.getFloat("y_strength", 1);
    configData.x_strength = preferences.getFloat("x_strength", 1);

    configData.fx_togoal = preferences.getFloat("fx_togoal", -0.2f);
    configData.fx_charge = preferences.getFloat("fx_charge", -0.4f);
    configData.fx_levy = preferences.getFloat("fx_levy", -0.1f);

    configData.n_max_x = preferences.getInt("n_max_x", 240);
    configData.n_max_y = preferences.getInt("n_max_y", 160);
    configData.h_ratio = preferences.getFloat("h_ratio", 0.75f);

    preferences.end();
}

// Implementation of getConfiguration
const nicla_t& NiclaConfig::getConfiguration() const {
    return configData;
}

// Provide access to DynamicConfig
hist_t* NiclaConfig::getDynamicHistory() {
    return &historyData;
}

// Implementation of getInstance
NiclaConfig* NiclaConfig::getInstance() {
    if (instance == nullptr) {
        instance = new NiclaConfig();
    }
    return instance;
}
