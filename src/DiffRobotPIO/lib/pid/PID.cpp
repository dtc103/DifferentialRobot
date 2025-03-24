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

    float p_value = error;

    this->integral += (error * dt);
    if(this->integral > (float)PID::MAX_INTEGRAL_VALUE){
        this->integral = (float)PID::MAX_INTEGRAL_VALUE;
    }else if(this->integral < (float)(-PID::MAX_INTEGRAL_VALUE)){
        this->integral = (float)(-PID::MAX_INTEGRAL_VALUE);
    }

    float derivative = (error - this->prev_error) / dt;
    this->prev_error = error;

    return this->p_gain * p_value + this->i_gain * this->integral + this->d_gain * derivative;
}

void PID::new_set_point(float set_point){
    this->set_point = set_point;
}

void PID::reset(){
    this->integral = 0.0;
    this->prev_error = 0.0;
}

