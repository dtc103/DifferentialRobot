#ifndef MPC_H
#define MPC_H

#include <Controller.h>
#include "QPSolver.h"

class MPC : public Controller {
public:
    MPC(int horizon, float model_dt, float model_a, float model_b,
        float cost_q, float cost_r, float cost_qf,
        float u_min, float u_max);

    MPC();

    float compute(const ControllerInput& input) override;
    void reset() override;

    void set_model(float a, float b);
    void set_costs(float q, float r, float qf);
    void set_limits(float u_min, float u_max);
    void set_horizon(int N);

private:
    // Model parameters
    int N; // prediction horizon
    float dt; // model time step (seconds)
    float a; // yaw rate decay
    float b; // input gain

    // Cost weights
    float q; // stage cost on theta error
    float r; // stage cost on input
    float qf; // terminal cost on theta error

    // Input constraints
    float u_min;
    float u_max;

    // Warm-start buffer
    float u_warm[MAX_QP_DIM];

    void build_qp(float theta0, float omega0, float theta_ref,
                   float* H, float* f_vec);

    static float wrap_angle(float angle);
};

#endif
