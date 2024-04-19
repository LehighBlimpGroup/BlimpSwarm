#pragma once

#include "state/IConfig.h"
#include <Preferences.h>

// nicla_t definition
typedef struct {
    int n_max_x, n_max_y;
    float fx_togoal, fx_charge, fx_levy;
    float h_ratio, y_thresh, y_strength, x_strength;
    float range_for_forward;
    int num_captures, time_in_ball, goal_height;
} nicla_t;

typedef struct {
    float last_detection_w = 0;
    float last_detection_h = 0;
    float last_tracking_x = 0;
    float last_tracking_y = 0;
    float des_yaw = 0;
    float robot_to_goal = 0;
    float z_estimator = 0;
    float forward_force = 0;
    int nicla_flag = 0x40;
    bool nicla_desired = 0;
    int num_captures = 0;
    unsigned long start_ball_time; 
} hist_t;

class NiclaConfig : public IConfig<nicla_t> {
    private:
        NiclaConfig();
        static NiclaConfig* instance;
        nicla_t configData;
        nicla_t configDatab;
        hist_t historyData;

    public:
        void loadConfiguration() override;
        const nicla_t& getConfiguration() const override;
        hist_t* getDynamicHistory();
        static NiclaConfig* getInstance();
};
