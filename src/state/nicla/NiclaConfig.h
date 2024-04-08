#pragma once

#include "state/IConfig.h"
#include <Preferences.h>

// nicla_t definition
typedef struct {
    int n_max_x, n_max_y;
    float fx_togoal, fx_charge, fx_levy;
    float h_ratio, y_thresh, y_strength, x_strength;
} nicla_t;

typedef struct {
    float last_detection_w = 0;
    float last_detection_h = 0;
    float last_tracking_x = 0;
    float last_tracking_y = 0;
    float des_yaw = 0;
    float robot_to_goal = 0;
    float z_estimator = 0;
} hist_t;

class NiclaConfig : public IConfig<nicla_t> {
    private:
        NiclaConfig();
        static NiclaConfig* instance;
        nicla_t configData;
        hist_t historyData;

    public:
        void loadConfiguration() override;
        const nicla_t& getConfiguration() const override;
        hist_t* getDynamicHistory();
        static NiclaConfig* getInstance();
};
