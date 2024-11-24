/**
 * @file DataTypes.h
 * @brief Contains all datatypes used throughout the project
 * @version 0.1
 * @date 2024-01-01
 * 
 * @copyright 
 * 
 */
#ifndef __DATA_TYPES_H__
#define __DATA_TYPES_H__

#include <cstdint>

// Define MAX_FLAGS
#define MAX_FLAGS 6

/**
 * @brief Structure to hold feedback parameters stored in the ESP32's flash memory.
 */
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

typedef struct nicla_t {
    bool state;
    int n_max_x, n_max_y;
    float fx_togoal, fx_charge, fx_levy;
    float h_ratio, y_thresh, y_strength, x_strength;
    float range_for_forward;
    int num_captures, time_in_ball, goal_height;
} nicla_t;


typedef struct hist_t {
    float last_detection_w;
    float last_detection_h;
    float last_tracking_x;
    float last_tracking_y;
    float des_yaw;
    float robot_to_goal;
    float z_estimator;
    float forward_force;
    int nicla_flag;
    bool nicla_desired;
    int num_captures;
    float goal_direction;
    unsigned long start_ball_time; 
} hist_t;


#endif // __DATA_TYPES_H__
