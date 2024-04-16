
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


    historyData.nicla_flag = preferences.getInt("state_flag", 0x40);

    configData.y_thresh = preferences.getFloat("y_thresh", 0.65);
    configData.y_strength = preferences.getFloat("y_strength", 1);
    configData.x_strength = preferences.getFloat("x_strength", 1);

    configData.fx_togoal = preferences.getFloat("fx_togoal", -0.2f);
    configData.fx_charge = preferences.getFloat("fx_charge", -0.4f);
    configData.fx_levy = preferences.getFloat("fx_levy", -0.1f);

    configData.n_max_x = preferences.getInt("bn_max_x", 240);
    configData.n_max_y = preferences.getInt("bn_max_y", 160);
    configData.h_ratio = preferences.getFloat("bh_ratio", 0.75f);

    configData.range_for_forward = preferences.getFloat("range_for_forward", 0.16);

    configDatab.y_thresh = preferences.getFloat("by_thresh", 0.65);
    configDatab.y_strength = preferences.getFloat("by_strength", 1);
    configDatab.x_strength = preferences.getFloat("bx_strength", 1);

    configDatab.fx_togoal = preferences.getFloat("bfx_togoal", -0.2f);
    configDatab.fx_charge = preferences.getFloat("bfx_charge", -0.4f);
    configDatab.fx_levy = preferences.getFloat("bfx_levy", -0.1f);

    configDatab.n_max_x = preferences.getInt("bn_max_x", 240);
    configDatab.n_max_y = preferences.getInt("bn_max_y", 160);
    configDatab.h_ratio = preferences.getFloat("bh_ratio", 0.75f);

    configDatab.range_for_forward = preferences.getFloat("brange_for_forward", 0.16);

    preferences.end();
}

// Implementation of getConfiguration
const nicla_t& NiclaConfig::getConfiguration() const {
    if (historyData.nicla_flag & 0x80){
        return configData;
    } else if (historyData.nicla_flag & 0x40){
        return configDatab;
    } else {
        return configDatab;// temp until more states
    }
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
