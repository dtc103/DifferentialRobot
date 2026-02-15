#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <Controller.h>

class PID : public Controller {
    public:
        static const int MAX_INTEGRAL_VALUE = 10000;

        PID();
        PID(float p, float i, float d, float initial_set_point);

        void set_p_gain(float);
        void set_i_gain(float);
        void set_d_gain(float);
        
        float update(float current_value);
        
        float compute(const ControllerInput& input) override;

        void new_set_point(float);
        void reset() override;

    private:
        float prev_error;
        float set_point;
        float integral;

        float p_gain;
        float i_gain;
        float d_gain;

        unsigned long last_time_update = 0;

        float compute_from_error(float error, float dt_ms);
};


#endif
