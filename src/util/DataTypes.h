#ifndef __DATA_TYPES_H__
#define __DATA_TYPES_H__
//
//typedef struct init_sensors_s {
//    float Kacc, Kgyro, Kmag;
//    bool baro;
//    float eulerGamma, rateGamma, zGamma;
//} init_sensors_t;
//
//
//typedef struct robot_specs_s {
//    int min_thrust, max_thrust;
//} robot_specs_s;
//
//typedef struct nicla_tuning_s  {
//    float goal_theta_back;
//    float goal_theta_front;
//    float goal_dist_thresh;
//    float max_move_x;
//    float goal_ratio;
//    float yaw_move_threshold;
//
//} nicla_tuning_s;
//
//typedef struct init_flags_s {
//    bool verbose, sensors, escarm, calibrate_esc, UDP, Ibus, ESPNOW, servo;
//    int PORT, motor_type, mode, control;
//} init_flags_t;
//
//typedef struct sensor_weights_t {
//    float eulerGamma;
//    float rollRateGamma, yawRateGamma, pitchRateGamma;
//    float zGamma, vzGamma;
//} sensor_weights_t;
//
//typedef struct sensors_s {
//    float roll, pitch, yaw;
//    float rollrate, pitchrate, yawrate;
//    float estimatedZ, velocityZ, groundZ;
//} sensors_t;
//
//typedef struct controller_s {
//    float fx;
//    float fy;
//    float fz;
//    float absz;
//    float tx;
//    float ty;
//    float tz;
//    bool ready;
//    int flag;
//    int snapshot;
//} controller_t;
//
//typedef struct raw_s {
//    float data[11];
//    bool ready;
//    int flag;
//
//} raw_t;
//
//typedef struct actuation_s {
//    float m1;
//    float m2;
//    float s1;
//    float s2;
//    bool ready;
//} actuation_t;
//
//

typedef struct feedback_t {
    bool zEn, yawEn, rollEn, pitchEn, rotateEn;
    float kpyaw, kppyaw, kdyaw, kddyaw, kiyaw, kiyawrate, yawRateIntRange;
    float kpz, kdz, kiz, z_int_low, z_int_high;
    float kproll, kdroll, kppitch, kdpitch;
    float servoBeta, servoRange, servo_move_min, botZlim, pitchOffset, pitchInvert;
    float lx;
} feedback_t;



enum DataType : uint8_t {
    DataType_Int = 0x01,
    DataType_Float = 0x02,
    DataType_String = 0x03,
    DataType_Bool = 0x04,
    // Add more datatypes as needed
};

typedef struct ControlInput {
    float params[13]; //FIXME magic number
} ControlInput;


typedef struct ReceivedData {
    int flag;
    float values[6];  //FIXME magic number
} ReceivedData;

// nicla_t definition
typedef struct {
    bool state;
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
    bool nicla_desired = 1;
    int num_captures = 0;
    float goal_direction = -5;
    unsigned long start_ball_time; 
} hist_t;

//
//typedef struct RollPitchAdjustments {
//    bool rollPitchSwitch;
//    float pitchSign, pitchOffset, rollSign, rollOffset;
//} RollPitchAdjustments;
//
//typedef struct randomwalk_values_t {
//    float forward_force, desired_z, desired_yaw, STEP_ZIG_ZAG;
//    int min_distance, NUM_ZIGS, Z_LEVEL, SWITCH_TIME;
//    int TIME_ROTATE, ANGLE_THRESH;
//    bool randomWalk_enabled;
//} randomwalk_values_t;


#endif