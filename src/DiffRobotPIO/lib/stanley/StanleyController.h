#ifndef STANLEY_CONTROLLER_H
#define STANLEY_CONTROLLER_H

#include <Controller.h>

class StanleyController : public Controller {
public:
    StanleyController(float k_heading, float k_crosstrack, float k_soft, float k_damp);
    StanleyController();

    float compute(const ControllerInput& input) override;
    void reset() override;

    void set_crosstrack_error(float error);

    // Parameter setters for runtime tuning
    void set_k_heading(float k);
    void set_k_crosstrack(float k);
    void set_k_soft(float k);
    void set_k_damp(float k);

private:
    float k_heading;
    float k_crosstrack;
    float k_soft;
    float k_damp;

    float crosstrack_error;  // updated externally when position sensors are available

    static float wrap_angle(float angle);
};

#endif
