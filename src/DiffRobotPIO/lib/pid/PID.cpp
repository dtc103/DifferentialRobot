#include "PID.h"

PID::PID() : p_gain(0.0), i_gain(0.0), d_gain(0.0){
    this->prev_error = 0.0;
    this->integral = 0.0;
    this->set_point = 0.0;
}

PID::PID(float p, float i, float d, float initial_set_point): p_gain(p), i_gain(i), d_gain(d){
    this->prev_error = 0.0;
    this->integral = 0.0;
    this->set_point = initial_set_point;
}

void PID::set_p_gain(float p_gain){
    this->p_gain = p_gain;
}

void PID::set_i_gain(float i_gain){
    this->i_gain = i_gain;
}

void PID::set_d_gain(float d_gain){
    this->d_gain = d_gain;
}

float PID::compute_from_error(float error, float dt_ms){
    float p_value = error;

    this->integral += (error * dt_ms);
    if(this->integral > (float)PID::MAX_INTEGRAL_VALUE){
        this->integral = (float)PID::MAX_INTEGRAL_VALUE;
    }else if(this->integral < (float)(-PID::MAX_INTEGRAL_VALUE)){
        this->integral = (float)(-PID::MAX_INTEGRAL_VALUE);
    }

    float derivative = (error - this->prev_error) / dt_ms;
    this->prev_error = error;

    return this->p_gain * p_value + this->i_gain * this->integral + this->d_gain * derivative;
}

float PID::update(float current_value){
    float curr_time = millis();
    float dt = curr_time - last_time_update;
    this->last_time_update = curr_time;

    float error = this->set_point - current_value;

    if(error > 180.0){
        error -= 360.0;
    }else if(error < -180.0){
        error += 360.0;
    }

    return compute_from_error(error, dt);
}

float PID::compute(const ControllerInput& input){
    // Use target_yaw as setpoint, current_yaw as measurement
    float error = input.target_yaw - input.current_yaw;

    // Wrap error for circular angles
    if(error > 180.0f){
        error -= 360.0f;
    }else if(error < -180.0f){
        error += 360.0f;
    }

    // Convert dt from seconds to milliseconds to match existing gain tuning
    float dt_ms = input.dt * 1000.0f;
    if(dt_ms < 0.001f) dt_ms = 1.0f; // guard against zero/negative dt

    return compute_from_error(error, dt_ms);
}

void PID::new_set_point(float set_point){
    this->set_point = set_point;
}

void PID::reset(){
    this->integral = 0.0;
    this->prev_error = 0.0;
}
