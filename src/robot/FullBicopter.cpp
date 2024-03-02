//
// Created by edward on 3/1/24.
//


#include "FullBicopter.h"
#include <Arduino.h>


#define SERVO1 D0
#define SERVO2 D1
#define THRUST1 D9
#define THRUST2 D10
#define BATT A2


FullBicopter::FullBicopter(): RawBicopter() {
    sensorsuite.startup();
    delay(500);
    
    float senses[MAX_SENSORS];
    FullBicopter::sense(senses);
    groundZ = senses[2];
}

int FullBicopter::sense(float sensors[MAX_SENSORS]) {
    sensorsuite.update();
    int numSenses = 0;
    //(pressure, temperature, altitude, roll, pitch, yaw, rollrate, pitchrate, yawrate, batteryVoltage)

    float* sensorsValues = sensorsuite.readValues(numSenses);
    for (int i = 0; i < numSenses; i++) {
        sensors[i] = sensorsValues[i];
    }
    // Implementation for sensing - fill the sensors array
    // Return the number of sensors used
    return numSenses; // Placeholder return value
}

// Controls [Ready, Fx, height/Fz, Tz, Tx]
bool FullBicopter::control(float sensors[MAX_SENSORS], float controls[], int size) {
    float outputs[5];
    if (controls[0] == 0) {
        outputs[0] = 0;
        outputs[1] = 0;
        outputs[2] = 90;
        outputs[3] = 90;
        outputs[4] = 0;
        
        return FullBicopter::actuate(outputs, size);
    }
    
    float feedbackControls[5];
    FullBicopter::addFeedback(sensors, controls, feedbackControls);
    
    outputs[4] = 1;
    FullBicopter::getOutputs(sensors, feedbackControls,  outputs);
    // Serial.print(outputs[0]);
    // Serial.print(",");
    // Serial.print(outputs[1]);
    // Serial.print(",");
    // Serial.print(outputs[2]);
    // Serial.print(",");
    // Serial.print(outputs[3]);
    // Serial.print(",");
    // Serial.println(outputs[4]);
    // delay(100);
    return FullBicopter::actuate(outputs, size);
}

void FullBicopter::getPreferences() {
    //calls the getPreferences of the superclass object to reduce the number of getPreferences calls
    RawBicopter::getPreferences();
    // Implementation for reading values from non-volatile storage (NVS)
    // must manually enter keys and default values for every variable.
    Preferences preferences; //initialize the preferences 
    preferences.begin("params", true); //true means read-only

    //value = preferences.getInt("value", default_value); //(value is an int) (default_value is manually set)
    
    // enables feedback on each vector
    PDterms.zEn = preferences.getBool("zEn", false);
    PDterms.yawEn = preferences.getBool("yawEn", false);
    PDterms.rollEn = preferences.getBool("rollEn", false);
    PDterms.rotateEn = preferences.getBool("rotateEn", false);
    PDterms.pitchEn = preferences.getBool("pitchEn", false);

    // PID terms
    PDterms.kpyaw = preferences.getFloat("kpyaw", 0.1);
    PDterms.kdyaw = preferences.getFloat("kdyaw", 0.1);// same thing as if I said kpyawrate
    PDterms.kiyaw = preferences.getFloat("kiyaw", 0);
    PDterms.kiyawrate = preferences.getFloat("kiyawrate", 0);

    PDterms.kpz = preferences.getFloat("kpz", 0.5);
    PDterms.kdz = preferences.getFloat("kdz", 0.5);
    PDterms.kiz = preferences.getFloat("kiz", 0);

    PDterms.kproll = preferences.getFloat("kproll", 0);
    PDterms.kdroll = preferences.getFloat("kdroll", 0);

    // Range terms for the integrals
    PDterms.z_int_low = preferences.getFloat("z_int_low", 0);
    PDterms.z_int_high = preferences.getFloat("z_int_high", 0);
    PDterms.yawRateIntRange = preferences.getFloat("yawRateIntRange", 0);

    // radius of the blimp
    PDterms.lx = preferences.getFloat("lx", .15);

    preferences.end();
}



//adds sensor feedback into the control values
void FullBicopter::addFeedback(float sensors[MAX_SENSORS], float controls[], float feedbackControls[]) {
    
    float fx = controls[1]; // Fx
    float fz = controls[2]; // Fz/height
    float tx = controls[3]; // tx
    float tz = controls[4]; // tz
    int dt = 4000;

    // Z feedback
    if (PDterms.zEn) {
        // Integral in Z
        z_integral += (controls[2] - (sensors[2]-groundZ)) * ((float)dt)/1000000.0f * PDterms.kiz;
        z_integral = clamp(z_integral, PDterms.z_int_low, PDterms.z_int_high);
        // Serial.println("z feedback");
        // PID in z
        fz = (controls[2] - (sensors[2]-groundZ))*PDterms.kpz 
                      - (0)*PDterms.kdz + (z_integral); //TODO velocityZ unimplemented
    }

    // Yaw feedback (Cascading control format) 
    //      this means that there is a separate PID for both absolute yaw and yawrate which are combined
    //      what this basically means is that the D term for the absolute yaw is replaced with the yawrate PI
    //          the D term in yaw is equivalent to the P term in yawrate
    if (PDterms.yawEn) {
        // Serial.println("yaw feedback");
        // initial absolute error for the yaw
        float e_yaw = controls[4] - sensors[5]; // Tz - Yaw
        e_yaw = atan2(sin(e_yaw), cos(e_yaw));
        e_yaw = clamp(e_yaw, -PI/3, PI/3);

        // integral term for the absolute yaw
        yaw_integral += e_yaw * ((float)dt)/1000000.0f * PDterms.kiyaw;
        yaw_integral = clamp(yaw_integral, -PI/4, PI/4);
        
        // finding desired yaw rate from the absolute yaw feedback
        float yaw_desired_rate = (e_yaw * PDterms.kpyaw + yaw_integral);

        // getting the error in the yawrate
        float e_yawrate = yaw_desired_rate - sensors[8]; // YawRate

        // integral term for the yawrate
        yawrate_integral += e_yawrate * ((float)dt)/1000000.0f * PDterms.kiyawrate;
        yawrate_integral = clamp(yawrate_integral, - PDterms.yawRateIntRange, PDterms.yawRateIntRange);

        // final result of the cascading PID controller in yaw
        tz = e_yawrate*PDterms.kdyaw + yawrate_integral;
    }

    // Roll feedback
    if (PDterms.rollEn) { 
        // Serial.println("roll feedback");
        tx = controls[3] - sensors[3]*PDterms.kproll - sensors[6] * PDterms.kdroll; // Roll - RollRate
    }

    // Roll and pitch rotation state feedback
    if (PDterms.rotateEn) {
        // Serial.println("rotate feedback");
        float cosp = cos(sensors[4]); // Pitch
        float sinp = sin(sensors[4]); // Pitch
        float cosr = cos(sensors[3]); // Roll
        float ifx = controls[1]; // Fx
        fx = ifx*cosp + fz*sinp;
        fz = (-1*ifx*sinp + fz* cosp)/cosr;
    }
    feedbackControls[0] = fx;
    feedbackControls[1] = fz;
    feedbackControls[2] = tx;
    feedbackControls[3] = tz;
    feedbackControls[4] = 1;
    
}


void FullBicopter::getOutputs(float sensors[MAX_SENSORS], float controls[], float out[]) {
    // Assuming PDterms, kf1, kf2, servo1offset, servo2offset, and feedbackPD.pitch are defined elsewhere

    float l = PDterms.lx;

    float fx = clamp(controls[0], -1, 1);
    float fz = clamp(controls[1], 0.001, 2);
    float taux = clamp(controls[2], -l + 0.01f, l - 0.01f);
    float tauz = clamp(controls[3], -1, 1);

    // Inverse A-Matrix calculations
    float term1 = l * l * fx * fx + l * l * fz * fz + taux * taux + tauz * tauz;
    float term2 = 2 * fz * l * taux - 2 * fx * l * tauz;
    float term3 = sqrt(term1 + term2);
    float term4 = sqrt(term1 - term2);
    float f1 = term3 / (2 * l);
    float f2 = term4 / (2 * l);
    float t1 = atan2((fz * l - taux) / term3, (fx * l + tauz) / term3);
    float t2 = atan2((fz * l + taux) / term4, (fx * l - tauz) / term4);
    
    if (PDterms.pitchEn) {
        t1 -= sensors[4]; // Pitch
        t2 -= sensors[4]; // Pitch
    }

    // Checking for full rotations and adjusting t1 and t2
    t1 = adjustAngle(t1);
    t2 = adjustAngle(t2);

    // Converting values to a more stable form
    out[2] = clamp((t1 ) / PI, 0.05f, 0.95f)* 180;
    out[3] = clamp((t2) / PI, 0.05f, 0.95f)* 180;
    out[0] = clamp(f1, 0, 1);
    out[1] = clamp(f2, 0, 1);

    // Adjust servo positions if motor speeds are too low
    if (out[0] < 0.02f) {
        out[2] = 90;
    }
    if (out[1] < 0.02f) {
        out[3] = 90;
    }
}

float FullBicopter::clamp(float val, float minVal, float maxVal) {
    return std::max(minVal, std::min(maxVal, val));
}

float FullBicopter::adjustAngle(float angle) {
  while (angle < -PI / 2) angle += 2 * PI;
  while (angle > 3 * PI / 2) angle -= 2 * PI;
  return angle;
}
