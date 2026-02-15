#ifndef CONTROLLER_H
#define CONTROLLER_H

//input structure for all controllers.
struct ControllerInput {
    float current_yaw;       
    float current_yaw_rate;
    float target_yaw;
    float velocity;
    float dt;
};

//Base class for all heading controllers.
class Controller {
public:
    virtual float compute(const ControllerInput& input) = 0;

    virtual void reset() = 0;

    virtual ~Controller() = default;
};

#endif
