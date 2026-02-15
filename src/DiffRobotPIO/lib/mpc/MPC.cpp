#include "MPC.h"
#include <math.h>
#include <string.h>

MPC::MPC()
    : N(5), dt(0.05f),
      a(0.8f), b(0.5f),
      q(10.0f), r(0.1f), qf(20.0f),
      u_min(-255.0f), u_max(255.0f) {
    memset(u_warm, 0, sizeof(u_warm));
}

// #TODO: Tune model parameters for robot.
//
//model_a (yaw rate decay, 0 < a < 1):
//model_b (input-to-yaw-rate gain):
//cost_q (heading error weight):
//cost_r (control effort weight):
//cost_qf (terminal cost weight):
MPC::MPC(int horizon, float model_dt, float model_a, float model_b,
         float cost_q, float cost_r, float cost_qf,
         float u_min, float u_max)
    : N(horizon > MAX_QP_DIM ? MAX_QP_DIM : horizon),
      dt(model_dt),
      a(model_a), b(model_b),
      q(cost_q), r(cost_r), qf(cost_qf),
      u_min(u_min), u_max(u_max) {
    memset(u_warm, 0, sizeof(u_warm));
}

float MPC::compute(const ControllerInput& input) {
    // Avoiding the 0/360 boundary issue in the prediction
    float theta0 = input.current_yaw;
    float omega0 = input.current_yaw_rate;
    float theta_ref = input.target_yaw;

    // Build QP matrices
    float H[MAX_QP_DIM * MAX_QP_DIM];
    float f_vec[MAX_QP_DIM];
    float lb[MAX_QP_DIM];
    float ub[MAX_QP_DIM];

    build_qp(theta0, omega0, theta_ref, H, f_vec);

    // Set box constraints
    for(int i = 0; i < N; i++){
        lb[i] = u_min;
        ub[i] = u_max;
    }

    // Use warm-start from previous solution (shifted by one step)
    float u_sol[MAX_QP_DIM];
    for(int i = 0; i < N - 1; i++){
        u_sol[i] = u_warm[i + 1];
    }
    u_sol[N - 1] = u_warm[N - 1];  // repeat last element

    // Solve QP
    QPSolver::solve(H, f_vec, lb, ub, u_sol, N, 50, 1e-3f);

    // Save solution for warm-starting next call
    memcpy(u_warm, u_sol, N * sizeof(float));

    // Apply first control action (receding horizon)
    return u_sol[0];
}

void MPC::reset() {
    memset(u_warm, 0, sizeof(u_warm));
}

void MPC::build_qp(float theta0, float omega0, float theta_ref,
                    float* H, float* f_vec) {

    float heading_diff = wrap_angle(theta_ref - theta0);
    float ref_unwrapped = theta0 + heading_diff;

    float a_pow[MAX_QP_DIM + 1];
    a_pow[0] = 1.0f;
    for(int k = 1; k <= N; k++){
        a_pow[k] = a_pow[k - 1] * a;
    }

    float sum_a[MAX_QP_DIM + 1];
    sum_a[0] = 0.0f;
    for(int k = 1; k <= N; k++){
        sum_a[k] = sum_a[k - 1] + a_pow[k - 1];
    }

    // Initialize H and f to zero
    memset(H, 0, N * N * sizeof(float));
    memset(f_vec, 0, N * sizeof(float));

    // Build H and f by iterating over prediction steps k = 1..N
    float phi_k[MAX_QP_DIM];

    for(int k = 1; k <= N; k++){
        // Free response
        float theta_k_free = theta0 + dt * omega0 * sum_a[k];
        float e_k = theta_k_free - ref_unwrapped;

        // Cost weight
        float w = (k == N) ? qf : q;

        for(int j = 0; j < N; j++){
            if(j >= k){
                phi_k[j] = 0.0f;
            } else {
                phi_k[j] = dt * b * sum_a[k - j - 1];
            }
        }

        // Accumulate into Hessian
        for(int i = 0; i < N; i++){
            for(int j = 0; j < N; j++){
                H[i * N + j] += w * phi_k[i] * phi_k[j];
            }
        }

        // Accumulate into linear cost
        for(int i = 0; i < N; i++){
            f_vec[i] += w * e_k * phi_k[i];
        }
    }

    // Add control effort cost to Hessian diagonal
    for(int i = 0; i < N; i++){
        H[i * N + i] += r;
    }
}

void MPC::set_model(float a, float b) {
    this->a = a;
    this->b = b;
}

void MPC::set_costs(float q, float r, float qf) {
    this->q = q;
    this->r = r;
    this->qf = qf;
}

void MPC::set_limits(float u_min, float u_max) {
    this->u_min = u_min;
    this->u_max = u_max;
}

void MPC::set_horizon(int N) {
    if(N > MAX_QP_DIM) N = MAX_QP_DIM;
    if(N < 1) N = 1;
    this->N = N;
}

float MPC::wrap_angle(float angle) {
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}
