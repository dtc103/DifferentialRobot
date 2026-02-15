#ifndef QP_SOLVER_H
#define QP_SOLVER_H

// Prediction horizon N. (Keep limited because of ESP32 stack limit)
static const int MAX_QP_DIM = 10;

class QPSolver {
public:
    static int solve(const float* H, const float* f,
                     const float* lb, const float* ub,
                     float* x, int n,
                     int max_iter = 50, float tol = 1e-4f);

private:
    // Compute g = H * x + f
    static void compute_gradient(const float* H, const float* f, const float* x,
                                 float* g, int n);

    // Clamp each element of x to [lb, ub].
    static void project(float* x, const float* lb, const float* ub, int n);

    // Compute squared L2 norm of a vector.
    static float norm_sq(const float* v, int n);
};

#endif
