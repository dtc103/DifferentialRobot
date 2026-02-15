#include "StanleyController.h"
#include <math.h>

StanleyController::StanleyController()
    : k_heading(1.0f),
      k_crosstrack(0.5f),
      k_soft(1.0f),
      k_damp(0.1f),
      crosstrack_error(0.0f) {}

// TODO: Tune these default values for your specific robot
// k_heading: 1.0
// k_crosstrack: needs encoders for crosstrack erroir
// k_soft: Prevents excessive correction at low speed
// k_damp: Yaw rate damping
StanleyController::StanleyController(float k_heading, float k_crosstrack, float k_soft, float k_damp)
    : k_heading(k_heading),
      k_crosstrack(k_crosstrack),
      k_soft(k_soft),
      k_damp(k_damp),
      crosstrack_error(0.0f) {}

float StanleyController::compute(const ControllerInput& input){
    // heading error term
    float heading_error = wrap_angle(input.target_yaw - input.current_yaw);

    // cross-track error term
    // when no position sensor, crosstrack_error stays at 0
    float velocity = input.velocity;
    float crosstrack_correction = 0.0f;
    if(fabsf(crosstrack_error) > 0.001f){
        crosstrack_correction = atan2f(k_crosstrack * crosstrack_error, velocity + k_soft);
        crosstrack_correction *= (180.0f / M_PI);
    }

    // yaw rate damping term for reduces oscillation
    float damping = k_damp * input.current_yaw_rate;

    // overall steering correction
    float correction = k_heading * heading_error + crosstrack_correction - damping;

    return correction;
}

void StanleyController::reset(){
    crosstrack_error = 0.0f;
}

void StanleyController::set_crosstrack_error(float error){
    this->crosstrack_error = error;
}

void StanleyController::set_k_heading(float k){
    this->k_heading = k;
}

void StanleyController::set_k_crosstrack(float k){
    this->k_crosstrack = k;
}

void StanleyController::set_k_soft(float k){
    this->k_soft = k;
}

void StanleyController::set_k_damp(float k){
    this->k_damp = k;
}

float StanleyController::wrap_angle(float angle){
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}
