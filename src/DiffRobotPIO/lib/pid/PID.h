#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID{
    public:
        static const int MAX_INTEGRAL_VALUE = 10000;

        PID();
        PID(float, float, float, float);

        void set_p_gain(float);
        void set_i_gain(float);
        void set_d_gain(float);
        
        float update(float);
        
        void new_set_point(float);
        void reset();

    private:
        float prev_error;
        float set_point;
        float integral;

        float p_gain;
        float i_gain;
        float d_gain;

        unsigned long last_time_update = 0;
};


#endif